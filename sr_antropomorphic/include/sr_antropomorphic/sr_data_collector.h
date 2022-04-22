#include <vector>
#include <iostream>
#include <map>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

class DataCollector
{
    public:

        DataCollector(int rate);

        void loop();

        void write_point_clouds();

    private:

        // Node handle
        ros::NodeHandle _nh;

        // Loop rate
        ros::Rate _rate;

        // tf buffer
        tf2_ros::Buffer _tf_buffer;

        // tf listener
        tf2_ros::TransformListener _tf_listener;

        // Dictionary of vectors of points
        std::shared_ptr<std::map<std::string, pcl::PointCloud<pcl::PointXYZ>>> _data;

        std::vector<std::string> _frame_names;

};
