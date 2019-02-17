#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

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

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(100);
  seg.setInputCloud (input_cloud);
  seg.segment (*floor_inliers, *coefficients);

  Eigen::Matrix<float, 1, 3> normal_vector_plane_tree, normal_vector_plane_xy, rotation_vector;

  normal_vector_plane_tree[0] = coefficients->values[0];
  normal_vector_plane_tree[1] = coefficients->values[1];
  normal_vector_plane_tree[2] = coefficients->values[2];

  std::cout << "\nnormal vector plane tree:\n" << normal_vector_plane_tree << std::endl;

  normal_vector_plane_xy[0] = 0.0;
  normal_vector_plane_xy[1] = 0.0;
  normal_vector_plane_xy[2] = 1.0;

  std::cout << "\nnormal vector plane XY:\n" << normal_vector_plane_xy << std::endl;

  rotation_vector = normal_vector_plane_tree.cross(normal_vector_plane_xy);
  std::cout << "\nrotation axis vector:\n "<< rotation_vector << std::endl;
  
  std::cout << "\nrotation axis module:\n "<< rotation_vector.norm() << std::endl;
  
  rotation_vector /= rotation_vector.norm();
  std::cout << "\nrotation axis normalized:\n "<< rotation_vector << std::endl;
  
  std::cout << "\nrotation axis normalized module:\n "<< rotation_vector.norm() << std::endl;

  float theta = acos((normal_vector_plane_xy.dot(normal_vector_plane_tree))/std::sqrt(std::pow(coefficients->values[0],2) +
                                                                                      std::pow(coefficients->values[1],2) + 
                                                                                      std::pow(coefficients->values[2],2)));
                                                                                      
  std::cout << "\nrotation angle(rad): " << theta << std::endl;  

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(theta, rotation_vector));
  
  std::cout << "\nTransformation matrix: " << "\n" << transform.matrix() << std::endl;
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

 
  viewer->addPointCloud(input_cloud, "original", PORT1);
  viewer->addPointCloud(output_cloud, "transform1 rvec", PORT2);

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

 
  viewer->setPosition(0,0);
  viewer->addCoordinateSystem();

  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "\nPress [q] to exit" << std::endl;

  while(!viewer->wasStopped ()) {
         viewer->spin();
  }
  
  pcl::io::savePCDFileBinary("cloud_alignmed.pcd",*output_cloud);
  
  return 0;
  
}




