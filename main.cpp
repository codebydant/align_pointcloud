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
  std::cout << "\nUse: " << progName << " <file>"  << std::endl <<
               "support: .pcd .ply .txt .xyz" << std::endl <<
               "[q] to exit" << std::endl;
}


int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;


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
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){
                  printUsage (argv[0]);
                  return -1;
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
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
      
    }else if(file_is_txt){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;
      unsigned int r, g, b; 

      while(file >> x_ >> y_ >> z_ >> r >> g >> b){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          uint8_t r_, g_, b_; 
          r_ = uint8_t(r); 
          g_ = uint8_t(g); 
          b_ = uint8_t(b); 

          uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_); 
          pt.rgb = *reinterpret_cast<float*>(&rgb_);               
              
          input_cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->points.size ());
      pcl::console::print_info (" points]\n");
      
    }else if(file_is_xyz){
  
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;

      while(file >> x_ >> y_ >> z_){
          
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          input_cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound xyz file.\n");
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
  seg.setDistanceThreshold(150);
  seg.setInputCloud (input_cloud);
  seg.segment (*floor_inliers, *coefficients);

  Eigen::Matrix<float, 1, 3> normal_vector_plane_tree, normal_vector_plane_xy, rotation_vector;
  
  
  Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;

    // Extract the eigenvalues and eigenvectors
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;

    pcl::compute3DCentroid(*input_cloud,centroid);
    

    
    


    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*input_cloud, centroid, covariance_matrix);
    pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);
    
    pcl::PointXYZ Point1 = pcl::PointXYZ((centroid(0) + eigen_vectors.col(0)(0)), (centroid(1) + eigen_vectors.col(0)(1)), (centroid(2) + eigen_vectors.col(0)(2)));
    
     // pcl::PointXYZ Point1 = pcl::PointXYZ((centroid(0)), (centroid(1) + eigen_vectors.col(0)(1)), (centroid(2)));

pcl::PointXYZ centroidXYZ;
centroidXYZ.getVector4fMap() = centroid;

pcl::PointXYZ Point2 = pcl::PointXYZ((centroid(0)), (centroid(1) + 2), (centroid(2)));
    
    

  //normal_vector_plane_tree[0] = coefficients->values[0];
  //normal_vector_plane_tree[1] = coefficients->values[1];
  //normal_vector_plane_tree[2] = coefficients->values[2];
  std::cout << "centroid point:" << centroid << std::endl;
  
  std::cout << "centroid pointXYZ:" << centroidXYZ << std::endl;
  std::cout << "new normal point:" << Point1 << std::endl;
  
  normal_vector_plane_tree[0] = Point1.x;
  normal_vector_plane_tree[1] = Point1.y;
  normal_vector_plane_tree[2] = Point1.z;

  std::cout << "\nnormal vector plane tree:\n" << normal_vector_plane_tree << std::endl;

  normal_vector_plane_xy[0] = 0.0;
  normal_vector_plane_xy[1] = 0.0;
  normal_vector_plane_xy[2] = 1.0;

  std::cout << "\nnormal vector plane XY:\n" << normal_vector_plane_xy << std::endl;

  rotation_vector = normal_vector_plane_xy.cross(normal_vector_plane_tree);
  std::cout << "\nrotation axis vector:\n "<< rotation_vector << std::endl;
  
  Eigen::Matrix<float, 1, 3> crooss_test;
  
  float dot_test = rotation_vector.dot(normal_vector_plane_tree);
  std::cout << "\nrotation axis vector test (dot=0):\n "<< dot_test << std::endl;
  dot_test = rotation_vector.dot(normal_vector_plane_xy);
  std::cout << "\nrotation axis vector test (dot=0):\n "<< dot_test << std::endl;
  
  std::cout << "\nrotation axis module:\n "<< rotation_vector.norm() << std::endl;
  
  rotation_vector /= rotation_vector.norm();
  std::cout << "\nrotation axis normalized:\n "<< rotation_vector << std::endl;
  
  std::cout << "\nrotation axis normalized module:\n "<< rotation_vector.norm() << std::endl;
  /*

  float theta = acos((normal_vector_plane_tree.dot(normal_vector_plane_xy))/std::sqrt(std::pow(normal_vector_plane_tree[0],2) +
                                                                                      std::pow(normal_vector_plane_tree[1],2) + 
                                                                                      std::pow(normal_vector_plane_tree[2],2)));
  */
                                                                                      
  float theta = acos((normal_vector_plane_xy.dot(normal_vector_plane_tree))/std::sqrt(std::pow(normal_vector_plane_tree[0],2) +
                                                                                      std::pow(normal_vector_plane_tree[1],2) + 
                                                                                      std::pow(normal_vector_plane_tree[2],2)));                                                                                    
                                                                                      
  std::cout << "\nrotation angle(rad): " << theta << std::endl;  
  float theta_deg = pcl::rad2deg(theta);
  
  std::cout << "\nrotation angle(deg): " << theta_deg << std::endl; 

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
  transform.translation() << 0.0, 0.0, 0.0;
  //transform.rotate(Eigen::AngleAxisf(theta, rotation_vector));
  
  Eigen::Vector3f axis(rotation_vector[0],rotation_vector[1],rotation_vector[2]);  
  Eigen::AngleAxis<float> rot(theta_deg,axis);
   transform.rotate(rot);
  
  std::cout << "\nTransformation matrix: " << "\n" << transform.matrix() << std::endl;
  pcl::transformPointCloud (*input_cloud, *output_cloud, transform);
  
       Eigen::Vector4f centroid2;
    Eigen::Matrix3f covariance_matrix2;

    // Extract the eigenvalues and eigenvectors
    Eigen::Vector3f eigen_values2;
    Eigen::Matrix3f eigen_vectors2;

    pcl::compute3DCentroid(*output_cloud,centroid2);
    
    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*output_cloud, centroid2, covariance_matrix2);
    pcl::eigen33 (covariance_matrix2, eigen_vectors2, eigen_values2);
  
  
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
  
  pcl::PointXYZ Point3 = pcl::PointXYZ((centroid2(0) + eigen_vectors2.col(0)(0)), (centroid2(1) + eigen_vectors2.col(0)(1)), (centroid2(2) + eigen_vectors2.col(0)(2)));
    
     // pcl::PointXYZ Point1 = pcl::PointXYZ((centroid(0)), (centroid(1) + eigen_vectors.col(0)(1)), (centroid(2)));

pcl::PointXYZ centroidXYZ2;
centroidXYZ2.getVector4fMap() = centroid2;


    
  pcl::PointXYZ centroidXYZ3;
centroidXYZ3.x = 0;
centroidXYZ3.y = 0;
centroidXYZ3.z = 0;

  pcl::PointXYZ punto5;
punto5.x = 0;
punto5.y = 0;
punto5.z = 1;
  
  viewer->addArrow(Point1,centroidXYZ , 0.5, 0.5, 0.5, false, "Arrow1",PORT1);
  viewer->addArrow(Point3,centroidXYZ2 , 0.5, 0.5, 0.5, false, "Arrow2",PORT2);
    viewer->addArrow(punto5,centroidXYZ3 , 0.5, 0.5, 0.5, false, "Arrow3",PORT1);

  //viewer->addLine<pcl::PointXYZRGB> (Point1, eigen_vectors, "line");

 
  viewer->setPosition(0,0);
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "\nPress [q] to exit" << std::endl;

  while(!viewer->wasStopped ()) {
         viewer->spin();
  }
  
  pcl::io::savePCDFileBinary("cloud_alignmed.pcd",*output_cloud);
  
  return 0;
  
}




