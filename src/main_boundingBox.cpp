     
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

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//Posted by Frogee April 30, 2015
//Find minimum oriented bounding box of point cloud (C++ and PCL)
//Here we're trying to get the minimum oriented bounding box of a point cloud using C++ and the Point Cloud Library (PCL). Most of the code originates from user Nicola Fioraio on the PCL forums in this post.
/*
Here is how user Nicola Fioraio describes the process:
    1) compute the centroid (c0, c1, c2) and the normalized covariance
    2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
    3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
    4) compute the max, the min and the center of the diagonal
    5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)

And here is my interpretation of the process along with the code. Here's the cloud we're starting with (it's a sorghum plant):

As I understand it, we first find the eigenvectors for the covariance matrix of the point cloud (i.e. principal component analysis, PCA). You'll replace the cloudSegmented pointer with a pointer to the cloud you want to find the oriented bounding box for.
*/


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace pcl;

int main (int argc,char* argv[]){

	 // The point clouds we will be using
	  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
	  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
	  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

	  // Checking program arguments
	  if (argc < 2)
	  {
	    printf ("Usage :\n");
	    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
	    PCL_ERROR ("Provide one ply file.\n");
	    return (-1);
	  }	

	pcl::console::TicToc time;
	  time.tic ();
	  if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
	  {
	    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
	    return (-1);
	  }
	  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;



	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_in, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
	                                                                                ///    the signs are different and the box doesn't get correctly oriented in some cases.
	/* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
	// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
	*/

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_in, *cloudPointsProjected, projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// This viewer has 4 windows, but is only showing images in one of them as written here.

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visu (new pcl::visualization::PCLVisualizer ("PlyViewer"));

	int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
	visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
	visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
	visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
	visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_in,30, 144, 255);
	visu->addPointCloud(cloud_in, color_handler, "bboxedCloud", mesh_vp_1);
	visu->addPointCloud(cloud_in, color_handler, "bboxedCloud3", mesh_vp_3);
	visu->addPointCloud(cloudPointsProjected, color_handler, "cloud_transformed", mesh_vp_2);
	visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
	visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox2", mesh_vp_4);
	visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"bbox2" ,mesh_vp_4);
	visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"bbox" ,mesh_vp_3);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler2(cloudPointsProjected,255, 255, 0);
visu->addPointCloud(cloudPointsProjected, color_handler2, "cloud_transformed2", mesh_vp_4);
 visu->addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  visu->addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  visu->addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  visu->addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
  visu->initCameraParameters();
  visu->resetCamera();

  pcl::console::print_info ("\npress [q] to exit!\n");

  while(!visu->wasStopped()){
      visu->spin();
  }


}