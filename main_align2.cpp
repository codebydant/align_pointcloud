     
#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h> 

 #include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace pcl;

int main (int argc,char* argv[]){
// The point clouds we will be using

 /////////////////////////////// Test //////////////////////
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  ground (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  centeredcloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  groundTransformed (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  final (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file .pcd \n");
        return (-1);
    }
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA>  seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*floor_inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(floor_inliers);
    extract.setNegative(false);
    extract.filter(*ground);
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }
    for(size_t i = 0; i < ground->points.size(); i++)
    {
        ground->points[i].r = 0;
        ground->points[i].g = 255;
        ground->points[i].b = 0;
    }
   
    double normalLength = sqrt((coefficients->values[0] * coefficients->values[0]) + (coefficients->values[1] * coefficients->values[1]) + (coefficients->values[2] * coefficients->values[2]));

    pcl::PointXYZ normalXY;
    pcl::PointXYZ normalGround;
    pcl::PointXYZ result;
    normalXY.x = 0.0;
    normalXY.y = 0.0;
    normalXY.z = 1.0;

    normalGround.x = coefficients->values[0]/normalLength;
    normalGround.y = coefficients->values[1]/normalLength;
    normalGround.z = coefficients->values[2]/normalLength;

    result.x = normalGround.y * normalXY.z - normalXY.y*normalGround.z;
    result.y = normalXY.x*normalGround.z - normalGround.x*normalXY.z;
    result.z = normalGround.x*normalXY.y - normalGround.y*normalXY.x;
    double resultLenght = sqrt((result.x * result.x) + (result.y * result.y) +(result.z * result.z));
    result.x /=resultLenght;
    result.y /=resultLenght;
    result.z /=resultLenght;
    double theta = std::acos(normalGround.z/sqrt((normalGround.x* normalGround.x)+(normalGround.y*normalGround.y)+(normalGround.z*normalGround.z)));
    //std::cout << "The crossproduct is " << result.x << " "<< result.y<< " "<< result.z <<std::endl;
    //std::cout << "norm " << std::sqrt((result.x * result.x  )+(result.y * result.y)+ (result.z * result.z))<< std::endl;

    double xx = (result.x * result.x) + ((1 - (result.x * result.x)) * std::cos(theta));
    double xy = (result.x * result.y)* (1 - std::cos(theta)) - (result.z * std::sin(theta));
    double xz = (result.x * result.z)* (1 - std::cos(theta)) + (result.y * std::sin(theta));
    double yx = (result.x * result.y)* (1 - std::cos(theta)) + (result.z * std::sin(theta));
    double yy = (result.y * result.y) + ((1 - (result.y * result.y)) * std::cos(theta));
    double yz = (result.y * result.z)* (1 - std::cos(theta)) - (result.x * std::sin(theta));
    double zx = (result.x * result.z)* (1 - std::cos(theta)) - (result.y * std::sin(theta));
    double zy = (result.y * result.z)* (1 - std::cos(theta)) + (result.x * std::sin(theta));
    double zz = (result.z * result.z) + ((1 - (result.z * result.z)) * std::cos(theta));
    Eigen::Matrix4f transformXY;
    transformXY = Eigen::Matrix4f::Identity();

    transformXY (0,0) = xx; //cos (theta);
    transformXY (0,1) = xy;// -sin(theta)
    transformXY (0,2) = xz;
    transformXY (1,0) = yx; // sin (theta)
    transformXY (1,1) = yy;
    transformXY (1,2) = yz;
    transformXY (2,0) = zx ;
    transformXY (2,1) = zy;
    transformXY (2,2) = zz;
    pcl::transformPointCloud (*cloud, *centeredcloud, transformXY);
    pcl::transformPointCloud (*ground, *groundTransformed, transformXY);
   
     Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid (*groundTransformed, xyz_centroid);
    std::cout << "The centroid is x " << xyz_centroid[0] << " y : "<< xyz_centroid[1] << " z " << xyz_centroid[2] << std::endl;
    Eigen::Matrix4f translationTransform = Eigen::Matrix4f::Identity();
    translationTransform = Eigen::Matrix4f::Identity();
    translationTransform (0,3) = - xyz_centroid[0];
    translationTransform (1,3) = - xyz_centroid[1];
    translationTransform (2,3) = - xyz_centroid[2];

//    translationTransform (0,3) = 0;
//    translationTransform (1,3) = 0;
//    translationTransform (2,3) = 0;

     pcl::transformPointCloud (*centeredcloud, *final, translationTransform);
    pcl::visualization::CloudViewer viewer ("Point Cloud Viewer");
        for(size_t i = 0; i < final->points.size(); i++)
    {
        final->points[i].r = 0;
        final->points[i].g = 255;
        final->points[i].b = 0;

    }
    viewer.showCloud (final);

    //viewer.showCloud (ground,"yolo");
    viewer.showCloud (centeredcloud,"yolo2");
    while (!viewer.wasStopped ())
    {
    } 

}
