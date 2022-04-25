#include <sr_antropomorphic/sr_surface_estimation.h>

SurfaceEstimation::SurfaceEstimation() :
_input_cloud(new pcl::PointCloud<pcl::PointXYZ>), _cloud_hull(new pcl::PointCloud<pcl::PointXYZ>)
{
    _reader.read("/home/user/projects/shadow_robot/base/pc_tests/rh_fftip.pcd", *_input_cloud);
}

void SurfaceEstimation::estimate_surface_pcl()
{
    // Create a Concave Hull representation of the projected inliers
    chull.setInputCloud(_input_cloud);
    chull.setAlpha(0.01);
    chull.reconstruct(*_cloud_hull);

    std::cerr << "Concave hull has: " << _cloud_hull->size ()
                << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("/home/user/projects/shadow_robot/base/pc_tests/concave_hull.pcd", *_cloud_hull, false);
}

void SurfaceEstimation::estimate_surface_open3d()
{

}

int main(int argc, char **argv)
{
    SurfaceEstimation surface_estimator;

    surface_estimator.estimate_surface_pcl();

    // Read Point Cloud
    std::cout << "Before reading pcd" << std::endl;
    io::ReadPointCloud("/home/user/projects/shadow_robot/base/pc_tests/rh_fftip.pcd", surface_estimator.input_cloud_2);

    std::cout << "PCD read" << std::endl;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh = geometry::TriangleMesh::CreateFromPointCloudAlphaShape(surface_estimator.input_cloud_2, 0.01);

    std::cout << "Draw geometries" << std::endl;
    visualization::DrawGeometries({mesh});
    
    return 0;
}