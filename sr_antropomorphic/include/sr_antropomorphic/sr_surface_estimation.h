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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include "open3d/Open3D.h"
#include "open3d/geometry/Geometry.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/visualization/visualizer/Visualizer.h"

using namespace open3d;


class SurfaceEstimation
{
    public:

        SurfaceEstimation();

        void estimate_surface_pcl();

        void estimate_surface_open3d();

        geometry::PointCloud input_cloud_2;

    private:

        pcl::PCDReader _reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _input_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_hull;
        pcl::ConcaveHull<pcl::PointXYZ> chull;

};