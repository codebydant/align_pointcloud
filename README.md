# align_pointcloud
Alignment between the coordinate system of a point cloud and the global coordinate system of pcl

----------------------
## Example

<img src="img.png" align="center" height="500" width="1000"><br>

## Compile
* Set "YOUR" PCL Build DIR in CMakeList.txt e.g: /opt/pcl-1.8.1/build and saved.
* Create a "build" folder

in the main folder:

	- cd /build  
	- cmake ../
    - make
       
        	 
### Test

	cd /build/bin
	./pcl-visualizer <pcd file> 
  	./pcl-visualizer <ply file> 



