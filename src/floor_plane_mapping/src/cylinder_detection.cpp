#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <Eigen/Core>


class CylinderDetection {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;
        visualization_msgs::MarkerArray ma;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        int _debug;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        pcl::PointCloud<pcl::PointXYZ> lastpc2_;
        cv::Mat_<uint32_t> accumulator;
        std::set<std::string> cylindersFound_;
        int min_votes_;
        double out_of_bounds_x_;
        double out_of_bounds_y_;
        int n_cx_, n_cy_, n_r_;
        int last_id_marker_ = 1;
        double cx_min, cx_max, cy_min, cy_max, r_min, r_max;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
            listener_.waitForTransform("bubbleRob",msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud("bubbleRob",msg->header.stamp, temp, msg->header.frame_id, lastpc2_, listener_);
            //
            if(_debug>0){
                ROS_INFO("Point Cloud Received");
            }
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
            
            //
            // BEGIN TODO
            // r^2 = (x - cx)^2 + (y - cy)^2 using the hough transform
            // Remember to use the cx_min,cx_max,n_cx_ variables (resp. b, c).
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            // fill the accumulator with zeros
            accumulator = 0;
            double delta_cx = (double) (cx_max - cx_min) / ((double) n_cx_);
            double delta_cy = (double) (cy_max - cy_min) / ((double)n_cy_);
            double delta_r = (r_max - r_min) / ((double)n_r_);
            if(_debug>0){
                ROS_INFO("delta_cx=%.5f, delta_cy=%.5f, delta_r=%.5f", delta_cx, delta_cy, delta_r);
            }
            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                for (int index_cx = 0; index_cx<n_cx_; index_cx++){
                    double cx = delta_cx * index_cx + cx_min;
                    for(int index_cy = 0; index_cy<n_cy_; index_cy++){
                        double cy = delta_cy * index_cy + cy_min;
                        double r = std::sqrt((x - cx)*(x - cx) + (y - cy)*(y - cy)) ;
                        int index_r = (int) ((r - r_min) / delta_r);
                        if(index_r < n_r_ && index_r >=0 ){
                            accumulator(index_cx, index_cy, index_r) += 1;
                            if(_debug>0){
                                //ROS_INFO("Voting for cx=%.2f, cy=%.2f, r=%.2f", cx, cy, r);
                            }
                        }
                    }
                    
                }
            }
            
	        //This is the worst implementation possible, please switch to an open cv function
            long maxAccum = 0;
            double X[3] = {1,1,1};
            std::string parametersKey = "";
            for (int index_a = 0; index_a<n_cx_; index_a++){
                double a = delta_cx * index_a + cx_min;
                for(int index_b = 0; index_b<n_cy_; index_b++){
                    double b = delta_cy * index_b + cy_min;
                    for(int index_c = 0; index_c < n_r_; index_c++){
                        double c = delta_r * index_c + r_min;
                        if(accumulator(index_a,index_b,index_c) > maxAccum){
                            maxAccum = accumulator(index_a,index_b,index_c);
                            X[0] = a;
                            X[1] = b;
                            X[2] = c;
                            parametersKey = std::to_string(index_a) + std::to_string(index_b) + std::to_string(index_c);
                            if(_debug>0){
                                //ROS_INFO("New max found at ");
                            }
                        }
                    }
                }
            }
            if(_debug>0){
                ROS_INFO("Cylinder found at cx=%.2f, cy=%.2f, r=%.2f with a value of %ld", X[0], X[1], X[2], maxAccum);
            }
            if(!cylindersFound_.count(parametersKey) && maxAccum > min_votes_){
                if(_debug>0){
                    ROS_INFO("New Cylinder found! adding it to the array");
                }
                cylindersFound_.insert(parametersKey);
                visualization_msgs::Marker m;
                m.header.stamp = msg->header.stamp;
                m.header.frame_id = base_frame_;
                m.ns = "cylinder_detection";
                m.id = last_id_marker_;
                m.type = visualization_msgs::Marker::CYLINDER;
                m.action = visualization_msgs::Marker::ADD;
                m.pose.position.x = X[0];
                m.pose.position.y = X[1];
                m.pose.position.z = 0;
                m.pose.orientation.x = 0;
                m.pose.orientation.y = 0;
                m.pose.orientation.z = 0;
                m.pose.orientation.w = 1;
                m.scale.x = X[2];
                m.scale.y = X[2];
                m.scale.z = 3;
                m.color.a = 0.5;
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 1.0;
                ma.markers.push_back(m);
                last_id_marker_++;
            }
            marker_pub_.publish(ma);
        }

    public:
        CylinderDetection() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("out_of_bounds_x", out_of_bounds_x_, 5.0);
            nh_.param("out_of_bounds_y", out_of_bounds_y_, 5.0);
            nh_.param("n_cx",n_cx_,10);
            nh_.param("cx_min",cx_min,-1.0);
            nh_.param("cx_max",cx_max,+1.0);
            nh_.param("n_cy",n_cy_,10);
            nh_.param("cy_min",cy_min,-1.0);
            nh_.param("cy_max",cy_max,+1.0);
            nh_.param("n_r",n_r_,10);
            nh_.param("r_min",r_min,-1.0);
            nh_.param("r_max",r_max,+1.0);
            nh_.param("min_votes", min_votes_, 100);
            nh_.param("debug", _debug, 0);
            assert(n_cx_ > 0);
            assert(n_cy_ > 0);
            assert(n_r_ > 0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("a: %d value in [%f, %f]",n_cx_,cx_min,cx_max);
            ROS_INFO("b: %d value in [%f, %f]",n_cy_,cy_min,cy_max);
            ROS_INFO("c: %d value in [%f, %f]",n_r_,r_min,r_max);

            // the accumulator is created here as a 3D matrix of size n_cx_ x n_cy_ x n_c
            int dims[3] = {n_cx_,n_cy_,n_r_};
            accumulator = cv::Mat_<uint32_t>(3,dims);
            
            // BEGIN TODO

            // You might want to add some pre-computation here to help working on the accumulator

            // END TODO


            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&CylinderDetection::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"cylinder_detection");
    CylinderDetection fp;

    ros::spin();
    return 0;
}
