#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <opencv2/opencv.hpp>

typedef std::pair<int,int> Coordinate;
typedef std::vector<pcl::PointXYZ> PointList;
typedef std::map<Coordinate,PointList> PointListMap;

class FloorPlaneMapping {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher occupancy_grid_pub_;
        tf::TransformListener listener_;
        int bayesian_filter_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        double out_of_bounds_x_;
        double out_of_bounds_y_;
        int point_list_max_size_;
        double occupancy_grid_resolution_;
        double real_map_width_meters_;
        double real_map_height_meters_;
        int occupancy_grid_width_;
        int occupancy_grid_height_;
        double traversable_threshold_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> lastpc2_;
        nav_msgs::OccupancyGrid occupancy_grid;

        cv::Mat_<double> PTxy;
        double DKnowingTxy [2][2] = {{0.9, 0.1}, {0.1, 0.9}};

    protected: // ROS Callbacks

        bool isTraversableNaive(PointList pointList){
            int n = pointList.size();
            Eigen::MatrixXf A(n,3);
            Eigen::MatrixXf B(n,1);
            for (int i = 0; i<n; i++) {
                pcl::PointXYZ P = pointList[i];
                double x = P.x;
                double y = P.y;
                double z = P.z;
                A(i,0) = x;
                A(i,1) = y;
                A(i,2) = 1;
                B(i,0) = z;
            }
            // Eigen operation on matrices are very natural:
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXf X = svd.solve(B);
            // z = ax + by + c <=> 0 = ax+by-1z+c
            //ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f", X(0),X(1),X(2));
            double acute_angle_cosine = std::abs((1.0)/std::sqrt(X(0)*X(0) + X(1)*X(1) + 1));
            //ROS_INFO("The slope of the floor plane is: %f", acute_angle);
            return acute_angle_cosine > traversable_threshold_;
        }

        int isTraversableBayesian(PointList pointList, int i, int j){
            int D = isTraversableNaive(pointList);
            int Txy = PTxy(i, j)>=0.5?1:0;
            double Txy1KnowingD = (DKnowingTxy[D][Txy] * PTxy(i, j)) / (DKnowingTxy[D][1]*PTxy(i, j) + DKnowingTxy[D][0]*(1.0-PTxy(i, j)));
            ROS_INFO("DKnowingTxy[D][1] = %.5f, PTxy(i, j)=%.5f, DKnowingTxy[D][0]=%.5f", DKnowingTxy[D][1], PTxy(i, j), DKnowingTxy[D][0]);
            ROS_INFO("D = %d, Txy = %d, Txy1KnowingD=%.5f", D, Txy, Txy1KnowingD);
            PTxy(i, j) = Txy1KnowingD;
            return 100 - (int) (Txy1KnowingD*100);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            // Receive the point cloud and convert it to the right format
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
            listener_.waitForTransform("bubbleRob",msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud("bubbleRob",msg->header.stamp, temp, msg->header.frame_id, lastpc2_, listener_);
            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                // In the sensor frame, this point would be inside the camera
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                // Measure the point distance in the base frame
                x = lastpc2_[i].x;
                y = lastpc2_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                if(std::abs(x) > out_of_bounds_x_ || std::abs(y) > out_of_bounds_y_){
                    //Point out of the map's bounds
                    continue;
                }
                // If we reach this stage, we have an acceptable point, so
                // let's store it
                pidx.push_back(i);
            }
            
            PointListMap pointListMap;
            int max_size = 1000;
            for (unsigned int i=0;i<pidx.size();i++) {
                pcl::PointXYZ P = lastpc_[pidx[i]];
                Coordinate C(round(P.x/occupancy_grid_resolution_), round(P.y/occupancy_grid_resolution_));
                if (pointListMap[C].size() < max_size) {
                    pointListMap[C].push_back(P);
                }
            }
            for (PointListMap::const_iterator it=pointListMap.begin();it!=pointListMap.end();it++) {
                const Coordinate & C = it->first;
                const PointList & pl = it->second;
                if (pl.empty()) {
                    continue;
                }
                int i = C.first + occupancy_grid_width_/2;
                int j = C.second + occupancy_grid_height_/2;
                if ((i < 0) || (j<0) || (i>=occupancy_grid_width_) || (j>=occupancy_grid_height_)) {
                    continue;
                }
                if(bayesian_filter_){
                    int traversable = isTraversableBayesian(pl, i, j);
                    occupancy_grid.data[j*occupancy_grid_height_ + i] = traversable;
                }else{
                    bool traversable = isTraversableNaive(pl);
                    occupancy_grid.data[j*occupancy_grid_height_ + i] = traversable?0:100;
                }
            }
            occupancy_grid_pub_.publish(occupancy_grid);
        }

    public:
        FloorPlaneMapping() : nh_("~") {
            // TODO START
            // The parameter below described the frame in which the point cloud
            // must be projected to be estimated. You need to understand TF
            // enough to find the correct value to update in the launch file
            nh_.param("base_frame",base_frame_,std::string("/body"));
            // This parameter defines the maximum range at which we want to
            // consider points. Experiment with the value in the launch file to
            // find something relevant.
            nh_.param("bayesian_filter",bayesian_filter_,1);
            nh_.param("max_range",max_range_,5.0);
            nh_.param("out_of_bounds_x", out_of_bounds_x_, 5.0);
            nh_.param("out_of_bounds_y", out_of_bounds_y_, 5.0);
            nh_.param("point_list_max_size", point_list_max_size_, 1000);
            nh_.param("real_map_width_meters", real_map_width_meters_, 10.0);
            nh_.param("real_map_height_meters", real_map_height_meters_, 10.0);
            nh_.param("occupancy_grid_resolution", occupancy_grid_resolution_, 1.0);
            nh_.param("traversable_threshold", traversable_threshold_, .1);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            occupancy_grid_width_ = real_map_width_meters_/occupancy_grid_resolution_;
            occupancy_grid_height_ = real_map_height_meters_/occupancy_grid_resolution_;
            occupancy_grid.header.stamp = ros::Time::now();
            occupancy_grid.header.frame_id = "world";
            occupancy_grid.info.resolution = occupancy_grid_resolution_;
            occupancy_grid.info.width = occupancy_grid_width_;
            occupancy_grid.info.height = occupancy_grid_height_;
            occupancy_grid.info.origin.position.x = -occupancy_grid_width_*occupancy_grid_resolution_/2;
            occupancy_grid.info.origin.position.y = -occupancy_grid_height_*occupancy_grid_resolution_/2;
            occupancy_grid.info.origin.position.z = 0;
            occupancy_grid.info.origin.orientation.w = 1;
            occupancy_grid.info.origin.orientation.x = 0;
            occupancy_grid.info.origin.orientation.y = 0;
            occupancy_grid.info.origin.orientation.z = 0;
            occupancy_grid.data.assign((int)(occupancy_grid_width_*occupancy_grid_height_),50);

            int dims[2] = {occupancy_grid_width_, occupancy_grid_height_};
            PTxy = cv::Mat_<double>(2,dims);
            PTxy = 0.5;

            // Subscribe to the point cloud and prepare the marker publisher
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneMapping::pc_callback,this);
            occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fp;

    ros::spin();
    return 0;
}


