RPL_individual_3: Object Recognition
================

README 

-- Functionality
This ROS node afford the user using PrimeSenseSensor kinect camera to detect object. The scene perceived from the camera is stored in image/complete_image in a .pcd file. Is then extracted and divided in clusters, and then saved in image/clusters_image. Each of the different cluster is then compared with the image stored in image/database_image. If the object is recognised by the program, it will get a colored marker visible in Rviz. 
* Red:
* Blu:
...


-- Usage
First start the ROS and rviz for the visualization (all in separate terminal):
is
	roscore

	rosrun rviz rviz

	roslaunch object_recognition object_recognition.launch

It is possible to modify all the global variable in the file parameters.yaml. !! Attention that the paths where the program is going to save some image will be cleaned from every other element.

-- File/directory structure
In the src and include directory:
* main: Here the core of the program run
* PointCloudH: this helper class contain function that make the work with pointcloud easier
* ClusterH: this class contain function that afford the "clusterization" of a pcd image
* DirectoriesParser: this class afford some operation on directories

-- Note for the instructor
I didn't use svn, but github for the commit, so if you are interested, please have a look at https://github.com/mafilipp/RPL_individual_3.
