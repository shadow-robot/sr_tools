#include "sr_antropomorphic/sr_data_collector.h"

#include <kdl_parser/kdl_parser.hpp>



DataCollector::DataCollector(int rate) :
_tf_listener(_tf_buffer), _rate(rate)
{

    // TODO Somehow obtain all interesting links
    _frame_names.push_back("rh_fftip");
    _frame_names.push_back("rh_ffdistal");
    _frame_names.push_back("rh_ffmiddle");
    _frame_names.push_back("rh_ffproximal");

    _frame_names.push_back("rh_lftip");
    _frame_names.push_back("rh_lfdistal");
    _frame_names.push_back("rh_lfmiddle");
    _frame_names.push_back("rh_lfproximal");

    _frame_names.push_back("rh_mftip");
    _frame_names.push_back("rh_mfdistal");
    _frame_names.push_back("rh_mfmiddle");
    _frame_names.push_back("rh_mfproximal");

    _frame_names.push_back("rh_rftip");
    _frame_names.push_back("rh_rfdistal");
    _frame_names.push_back("rh_rfmiddle");
    _frame_names.push_back("rh_rfproximal");

    _frame_names.push_back("rh_thtip");
    _frame_names.push_back("rh_thdistal");
    _frame_names.push_back("rh_thmiddle");

    _data = std::make_shared<std::map<std::string, pcl::PointCloud<pcl::PointXYZ>>>();

    for (auto& link_name : _frame_names)
    {   
        _data->insert(std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>>(link_name, pcl::PointCloud<pcl::PointXYZ>()));
    }

}

void DataCollector::loop()
{
    int samples = 0;

    while (_nh.ok() && samples < 10000)
    {
        for (auto& frame : _frame_names)
        {
            geometry_msgs::TransformStamped transformStamped;

            try
            {
                transformStamped = _tf_buffer.lookupTransform("rh_palm", frame,
                                                            ros::Time(0));

                // TODO Check previous value is different
                if (samples > 0)
                {
                    if (_data->at(frame).back().x == transformStamped.transform.translation.x)
                        continue;
                }

                pcl::PointXYZ point(transformStamped.transform.translation.x,
                                    transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.z);

                _data->at(frame).push_back(point);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

        }
        _rate.sleep();

        std::cout << "Samples: " << samples++ << std::endl;
    }
}

void DataCollector::write_point_clouds()
{
    for (auto& field : *_data)
    {
        // Store data like this?
        pcl::io::savePCDFileASCII ("/home/user/projects/shadow_robot/base/pc_tests/" + field.first + ".pcd", field.second);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_collector");

    DataCollector data_collector(1000);

    ros::Duration(5.0).sleep();

    data_collector.loop();

    data_collector.write_point_clouds();

    return 0;
}