#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneHough {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;
        int _debug;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        cv::Mat_<uint32_t> accumulator;

        int n_a, n_b, n_c;
        double a_min, a_max, b_min, b_max, c_min, c_max;

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            
            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // transformed to c = z - a*x - b*y
            // Remember to use the a_min,a_max,n_a variables (resp. b, c).
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            // fill the accumulator with zeros
            accumulator = 0;
            double delta_a = (a_max - a_min) / ((double) n_a);
            double delta_b = (b_max - b_min) / ((double)n_b);
            double delta_c = (c_max - c_min) / ((double)n_c);
            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                for (int index_a = 0; index_a<n_a; index_a++){
                    double a = delta_a * index_a + a_min;
                    for(int index_b = 0; index_b<n_b; index_b++){
                        double b = delta_b * index_b + b_min;
                        double c = z - a * x - b * y;
                        int index_c = (int) ((c - c_min) / delta_c);
                        accumulator(index_a, index_b, index_c) += 1;
                        if(_debug>1){
                            ROS_INFO("Value of a=%.2f, b=%.2f, c=%.2f",a, b, c);
                            ROS_INFO("Accumulatior(%d, %d, %d)=%d",index_a, index_b, index_c, accumulator(index_a, index_b, index_c));
                        }
                    }
                    
                }
            }
            
	        //This is the worst implementation possible, please switch to an open cv function
            long maxAccum = 0;
            double X[3] = {1,1,1};
            for (int index_a = 0; index_a<n_a; index_a++){
                double a = delta_a * index_a + a_min;
                for(int index_b = 0; index_b<n_b; index_b++){
                    double b = delta_b * index_b + b_min;
                    for(int index_c = 0; index_c < n_c; index_c++){
                        double c = delta_c * index_c + c_min;
                        if(accumulator(index_a,index_b,index_c) > maxAccum){
                            maxAccum = accumulator(index_a,index_b,index_c);
                            X[0] = a;
                            X[1] = b;
                            X[2] = c;
                            if(_debug>0){
                                ROS_INFO("New max found at a=%.2f, b=%.2f, c=%.2f with a score of %ld", X[0], X[1], X[2], maxAccum);
                            }
                        }
                    }
                }
            }
            
            // Use the accumulator to find the best plane parameters and store
            // them in X (this will be used for display later)
            // X = {a,b,c}

            // END OF TODO
            ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            
            visualization_msgs::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            tf::quaternionTFToMsg(Q,m.pose.orientation);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            marker_pub_.publish(m);
            
        }

    public:
        FloorPlaneHough() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_a",n_a,10);
            nh_.param("a_min",a_min,-1.0);
            nh_.param("a_max",a_max,+1.0);
            nh_.param("n_b",n_b,10);
            nh_.param("b_min",b_min,-1.0);
            nh_.param("b_max",b_max,+1.0);
            nh_.param("n_c",n_c,10);
            nh_.param("c_min",c_min,-1.0);
            nh_.param("c_max",c_max,+1.0);
            nh_.param("debug", _debug, 0);
            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_c > 0);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("a: %d value in [%f, %f]",n_a,a_min,a_max);
            ROS_INFO("b: %d value in [%f, %f]",n_b,b_min,b_max);
            ROS_INFO("c: %d value in [%f, %f]",n_c,c_min,c_max);

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_c};
            accumulator = cv::Mat_<uint32_t>(3,dims);
            
            // BEGIN TODO

            // You might want to add some pre-computation here to help working on the accumulator

            // END TODO


            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneHough::pc_callback,this);
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_Hough");
    FloorPlaneHough fp;

    ros::spin();
    return 0;
}


