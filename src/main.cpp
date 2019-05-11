#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/feature.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>

#include <iostream>

void printUsage (const char* progName){
  std::cout << "\nUsage: " << progName << " <file.pcd> or <file.ply>"  << std::endl <<
               "[q] to exit" << std::endl;
}


int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;


  if(argc < 2 or argc > 2){
      printUsage (argv[0]);
      return -1;
  }

  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
           std::cout << "ply or pcd file no valid!" << std::endl;
           return -1;
      }else if(filenames.size() == 1){
          file_is_pcd = true;
      }
  }else if(filenames.size() == 1){
      file_is_ply = true;
  }else{
      printUsage (argv[0]);
      return -1;
  }

  if(file_is_pcd){
      if(pcl::io::loadPCDFile(argv[filenames[0]], *input_cloud) < 0){
          std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
          printUsage (argv[0]);
          return -1;
      }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*input_cloud);
      if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *input_cloud);
          if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*input_cloud);
              if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return -1;
              }
          }
       }

      pcl::console::print_info("\nFound ply file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->points.size ());
      pcl::console::print_info (" points]\n");
    }
      
  input_cloud->width = (int) input_cloud->points.size ();
  input_cloud->height = 1;
  input_cloud->is_dense = true;
  
  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */
  
  Eigen::Matrix<float, 3, 1> obj_vec, xy_vec, axis;

  xy_vec[0] = 0.0;
  xy_vec[1] = 0.0;
  xy_vec[2] = 1.0;
  
  std::cout << "\nXY vector:\n" << xy_vec << std::endl;
  
  /*
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(150);
  seg.setInputCloud (input_cloud);
  seg.segment (*floor_inliers, *coefficients);
  */
  
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  // Extract the eigenvalues and eigenvectors
  Eigen::Vector3f eigen_values;
  Eigen::Matrix3f eigen_vectors;

  pcl::compute3DCentroid(*input_cloud,centroid);
  
 
    
  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix (*input_cloud, centroid, covariance_matrix);
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);
  
   /*
  
  pcl::PointXYZ centroidXYZ;
  centroidXYZ.getVector4fMap() = centroid;
  */
  //pcl::PointXYZ Point1 = pcl::PointXYZ((eigen_vectors.col(0)(0)), (eigen_vectors.col(0)(1)), (eigen_vectors.col(0)(2)));
  pcl::PointXYZ Point1 = pcl::PointXYZ((centroid(0)), (centroid(1)), (centroid(2)));

  
  
  //obj_vec[0] = coefficients->values[0];
  //obj_vec[1] = coefficients->values[1];
  //obj_vec[2] = coefficients->values[2];
 
  
  Eigen::Vector4f centroid_arrow;
  centroid_arrow[0] = centroid(0);
  centroid_arrow[1] = centroid(1)+2;
  centroid_arrow[2] = centroid(2);
  

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (input_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);
  

  // Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
  std::vector<int> indices (floor (input_cloud->points.size ()));
  for (size_t i = 0; i < indices.size (); ++i) indices[i] = i;
  
  Eigen::Vector4f plane_parameters;
  float curvature;
  ne.computePointNormal (*input_cloud, indices, plane_parameters, curvature);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  
  
  //obj_vec[0] = centroid[0];
  //obj_vec[1] = centroid[1];
  //obj_vec[2] = centroid[2];
  
  //obj_vec[0] = (centroid_arrow[0] - centroid[0]);
  //obj_vec[1] = -(centroid_arrow[1] - centroid[1]);
  //obj_vec[2] = (centroid_arrow[2] - centroid[2]);
  
  //obj_vec[0] = (centroid[0] + eigen_vectors.col(0)(0));
  //obj_vec[1] = -(centroid[1] + eigen_vectors.col(0)(1));
  //obj_vec[2] = (centroid[2] + eigen_vectors.col(0)(2));
  
  // http://pointclouds.org/documentation/tutorials/normal_estimation.php
  
  //obj_vec[0] = (eigen_vectors.col(0)(0));
  //obj_vec[1] = -(eigen_vectors.col(0)(1));
  //obj_vec[2] = (eigen_vectors.col(0)(2));
  
  obj_vec[0] = (plane_parameters[0]);
  if(plane_parameters[1]<0){
       obj_vec[1] = -(plane_parameters[1]);
  }else{
      obj_vec[1] = (plane_parameters[1]);
  }

  obj_vec[2] = (plane_parameters[2]);
  
  
  
  //obj_vec[0] = 2;
  //obj_vec[1] = 2;
  //obj_vec[2] = 2;
  
  std::cout << "\nobjec vector:\n" << obj_vec << std::endl;

  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */

  axis = obj_vec.cross(xy_vec); 
  std::cout << "\naxis:\n "<< axis << std::endl;
  
  Eigen::Matrix<float, 3, 1> crooss_test;
  
  float dot_test = axis.dot(obj_vec);
  std::cout << "\naxis vector test (dot=0):\n "<< dot_test << std::endl;
  dot_test = axis.dot(xy_vec);
  std::cout << "\naxis vector test (dot=0):\n "<< dot_test << std::endl;
  
  std::cout << "\naxis vector module:\n "<< axis.norm() << std::endl;
  
  axis /= axis.norm();
  std::cout << "\naxis normalized:\n "<< axis << std::endl;
  
  std::cout << "\naxis normalized module:\n "<< axis.norm() << std::endl;
  
  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */
  
  /*

  float theta = acos((normal_vector_plane_tree.dot(normal_vector_plane_xy))/std::sqrt(std::pow(normal_vector_plane_tree[0],2) +
                                                                                      std::pow(normal_vector_plane_tree[1],2) + 
                                                                                      std::pow(normal_vector_plane_tree[2],2)));
  */
                                                                                      
  float theta = acos((obj_vec.dot(xy_vec))/std::sqrt(std::pow(obj_vec[0],2) + std::pow(obj_vec[1],2) + std::pow(obj_vec[2],2)));                                                                                    
                                                                                      
  std::cout << "\nangle(rad): " << theta << std::endl;  
  float theta_deg = pcl::rad2deg(theta);
  
  std::cout << "\nangle(deg): " << theta_deg << std::endl; 
  
  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */
  
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << 0.0, 0.0, 0.0;
  transform.rotate(Eigen::AngleAxisf(theta, axis));
  
  //Eigen::Matrix3f rotation_matrix;
  
  //rotation_matrix = Eigen::AngleAxisf(theta, axis); 
    
  //std::cout << "\nTransformation matrix: " << "\n" << rotation_matrix.matrix() << std::endl;
  std::cout << "\nTransformation matrix: " << "\n" << transform.matrix() << std::endl;
  
  /* ----------------------------------------------------------------------- */
  /* ----------------------------------------------------------------------- */ 
  
  Eigen::Matrix<float, 3, 1> vec_transformed;
  
  //vec_transformed = rotation_matrix * obj_vec;  

  //std::cout << "\nPoint transformed: " << "\n" << vec_transformed << std::endl;
  
  pcl::transformPointCloud (*input_cloud, *output_cloud, transform);
  

  
   /*************************
  PCL VISUALIZER
  **************************/
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor (0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor (0, 0, 0, PORT2);
  viewer->addText("ALIGNMENT AXIS", 10, 10, "PORT2", PORT2);
  
  viewer->removeAllPointClouds(0);

  if(input_cloud->points[0].r <= 0 and input_cloud->points[0].g <= 0 and input_cloud->points[0].b<= 0 ){
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(input_cloud,255,255,0);      
      viewer->addPointCloud(input_cloud,color_handler,"Original",PORT1);
  }else{
      viewer->addPointCloud(input_cloud,"Original",PORT1);
  }
  
  if(output_cloud->points[0].r <= 0 and output_cloud->points[0].g <= 0 and output_cloud->points[0].b<= 0 ){
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(output_cloud,255,255,0);
      viewer->addPointCloud(output_cloud,color_handler,"transform1 rvec",PORT2);
  }else{
      viewer->addPointCloud(output_cloud,"transform1 rvec",PORT2);
  }
  

 

  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer->addCoordinateSystem(1,"original_usc",PORT1);
  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_",PORT1);
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_",PORT1);
  viewer->addText3D ("z", p3,0.2, 0, 0, 1, "z_",PORT1);

  viewer->addCoordinateSystem(1,"transform_ucs",PORT2);
  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_",PORT2);
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_",PORT2);
  viewer->addText3D ("z", p3,0.2, 0, 0, 1, "z_",PORT2);
  
  pcl::PointXYZ origen;
  origen.x = 0;
  origen.y = 0;
  origen.z = 0;
  
  Eigen::Vector4f obj_vec_transformed;
  float curvature_obj_vec;
  ne.computePointNormal (*output_cloud, indices, obj_vec_transformed, curvature_obj_vec);
  
  pcl::PointXYZ xy_vecXYZ = pcl::PointXYZ(xy_vec(0), xy_vec(1), xy_vec(2));
  pcl::PointXYZ obj_vecXYZ = pcl::PointXYZ(obj_vec(0), obj_vec(1), obj_vec(2));
  pcl::PointXYZ axis_vecXYZ = pcl::PointXYZ(axis(0), axis(1), axis(2));
  pcl::PointXYZ vec_transformedXYZ = pcl::PointXYZ(obj_vec_transformed(0), obj_vec_transformed(1), obj_vec_transformed(2));
  
  pcl::PointXYZ origen_desf;
  origen_desf.x = 0;
  origen_desf.y = 0;
  origen_desf.z = 0;
  
    
  
  viewer->addArrow(xy_vecXYZ,origen , 255.0, 255.0, 0.0, false, "Arrow1",PORT1);
  viewer->addArrow(obj_vecXYZ,origen_desf , 0.0, 255.0, 0.0, false, "Arrow2",PORT1);
  viewer->addArrow(axis_vecXYZ,origen , 0.0, 0.0, 255.0, false, "Arrow3",PORT1);
  viewer->addArrow(vec_transformedXYZ,origen , 255.0, 255.0, 255.0, false, "Arrow4",PORT2);

  //viewer->addLine<pcl::PointXYZRGB> (Point1, eigen_vectors, "line");

 
  viewer->setPosition(0,0);
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "\nPress [q] to exit" << std::endl;

  while(!viewer->wasStopped ()) {
         viewer->spin();
  }

  
  return 0;
  
}




