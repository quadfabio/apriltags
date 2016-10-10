apriltags
=========

*This repository is a fork of https://github.com/personalrobotics/apriltags ROS wrapper for the Swatbotics C++ port of the AprilTag visual fiducial detector http://github.com/swatbotics/apriltags-cpp, with additional features.*

ROS wrapper for the Swatbotics C++ port of the AprilTag visual fiducial detector with localization algorithm for TurtleBot.  
The Swatbotics port uses OpenCV and CGAL for improved performance.  

**Installation**

Install dependencies:  
> $ sudo aptitude install libcgal-dev

Clone this repo into your catkin workspace and build.

Usage
=====
Mapping
--------
**Edit the launch file `make_map.launch`.**    

Define max front distance (max_dist) and max side distance (max_o_dist) of tag detections you want to consider as valid.   

> `<name="max_d" default="1.5"/>`          
> `<name="max_o_d" default="0.5"/>`

Set apriltags node parameters:    
>##### Default tag size as the width of the black square you printed out.
> `<param name="~default_tag_size" value="0.046"/>`    
>##### Window with video stream and tag detection  
> `<param name="~viewer" value="false"/>`   
>##### Republish modified image with tag detection   
> `<param name="~publish_detections_image" value="true"/>`    
>##### Type and thickness of tag detection marker on RViz 
> `<param name="~display_type" value="CUBE"/>`     
> `<param name="~marker_thickness" value="0.02"/>`
>##### If your webcam does not publish images to the default topic name, you may need to edit these 2 parameters:  
> `<remap from="~image" to="/camera/rgb/image_rect"/>`     
> `<remap from="~camera_info" to="/camera/rgb/camera_info"/>`  

Set write_markers node parameters:    
>##### Path for the file where save tags position  
> `<param name="~yaml_file_path" value="/home/orobot/workspace/ros/catkin/src/apriltags/include/detected_tag.yaml"/>`   
>##### Frames for map and camera    
> `<param name="~map_frame" value="map"/>`   
> `<param name="~camera_frame" value="kinect2_head_rgb_optical_frame"/>`    
>##### Tell the node wich is the maximum id it will find in your room, to avoid false tag detection   
> `<param name="~max_id" value="10"/>`   

**Terminal**

Launch all the nodes    
> $ roslaunch apriltags make_map.launch  

And drive the robot through your room. For a better result keep a low speed and remains perfectly in front of every tag for few seconds (~5-10 secs).   
RViz will automatically startup with a specific configuration. You can see the Kinect video stream on the left part of the screen. Every detection will be showed, but only the ones under the thresholds setted will have coloured contours.   
When you're satisfied open a new terminal and save your map:   
> $ rosrun map_server map_saver -f /home/orobot/workspace/ros/catkin/src/apriltags/include/maps/**map_name**  

and shutdown the nodes.
Navigation
----------
**Edit the launch file `tbl.launch`.**   

Set all the parameters for apriltags node as done before.  

Set estimated_localization node parameters, choosing which tags you want to load. These are similiar to the ones for write_markers node, except for:    
>##### Visualize an additional marker for base frame during estimation of the pose   
>`<param name="~vis_base_marker" value="false" />`   
>##### Visualize the markers you saved during mapping    
>`<param name="~vis_real_markers" value="true" /> `           

Set Tag Based Localization (tbl) node parameters:   
>##### Define the rate for odom pose to be published   
>`<param name="~rate" value="10" />`   
>##### Define start delay for the algorithm to start    
>`<param name="~start_delay" value="2" />`     
>##### Define the minimun difference between the pose given by odom information and the one estimated   
>`<param name="~distance_threshold" value="0.5" /> `   
>##### Define the minimun difference between the current and last pose    
>`<param name="~precision" value="0.3" />`

Choose the map you want to load:   
>`<node name="map_server" pkg="map_server" type="map_server" args="/home/orobot/workspace/ros/catkin/src/apriltags/include/maps/map_with_april_tags.yaml" />`

**Terminal**

Launch all the nodes    
> $ roslaunch apriltags tbl.launch   
