/*
	This node subscribes to /apriltags/marker_array topic, saving every tag detections.
	When shutted down, it calculates the mean value for every tag detected and write it on an YAML file.

    TODO: move the reference frame change off-line. Just save every tag position in camera reference frame during mapping.
*/

//		Libraries

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vis_msgs_marker.tpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "yaml-cpp/yaml.h"
#include "write_markers.h"
#include <sstream>
#include <fstream>

#include <signal.h>

//  Override shutdown function.
void mySigintHandler(int sig)
{
	
	ROS_WARN("Writing detections");
	WriteDetections();

    ROS_WARN("Shutting down WriteMarkers node.");
    ros::shutdown();
}

//      Callbacks

//  Read detected tag from topic /apriltags/marker_array
void MarkerCallback (const visualization_msgs::MarkerArray& published_markers)
{
	//	marker writable copy.
	visualization_msgs::Marker marker;

	for (int i = 0; i < published_markers.markers.size(); ++i)
	{

		//	Avoid false tag detection
		if(published_markers.markers[i].id > max_id_)
			continue;
		
		tf::Transform tf_map2kinect;
		
		double dx = published_markers.markers[i].pose.position.x;
		double dy = published_markers.markers[i].pose.position.y;
		double dz = published_markers.markers[i].pose.position.z;
		
		double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        
        //  Use only near tag
        if(distance > max_dist_)
        {
            continue;
        }

        //  Checking for angles only for near tags
        double yaw = tf::getYaw(published_markers.markers[i].pose.rotation);
        double pitch = tf::getPitch(published_markers.markers[i].pose.rotation);
        
        if(abs(yaw) > max_angle_ || abs(pitch) > max_angle_)    
        {
            continue;
        }

        //  Use only near tags and check for transform to exist.  
		//if (distance < max_dist_ && GetTf(map_frame_, published_markers.markers[i].header.frame_id, tf_map2kinect, 0.2))
        if (distance < max_dist_ && GetTf(map_frame_, camera_frame_, tf_map2kinect, 0.2))
        {
        	//	make a copy only if necessary.
        	marker = published_markers.markers[i];
	        
            // create a geometry_msgs::Transform variable from visualization_msgs::Marker and convert it to tf::Transform 
            tf::Transform tf_kinect2tag;
	        geometry_msgs::Transform tag_msg;

	        tag_msg.rotation = marker.pose.orientation;
	        tag_msg.translation.x = marker.pose.position.x;
	        tag_msg.translation.y = marker.pose.position.y;
	        tag_msg.translation.z = marker.pose.position.z;
	        
	        tf::transformMsgToTF(tag_msg, tf_kinect2tag);
            //  changing reference frame from camera_frame to map_frame
	        tf_kinect2tag = tf_map2kinect*tf_kinect2tag;
	        
            //  converting tf::Transform to geometry_msgs::Transform
	        tf::transformTFToMsg(tf_kinect2tag, tag_msg);
	        marker.pose.orientation = tag_msg.rotation;
	        marker.pose.position.x = tag_msg.translation.x;
	        marker.pose.position.y = tag_msg.translation.y;
	        marker.pose.position.z = tag_msg.translation.z;

	        marker.header.frame_id = map_frame_;
			
			//   pushing in the array the marker.
			detected_tags_array_[marker.id].markers.push_back(marker);
		}
	}
}

//		Functions

//  Returns if the transform from origin frame to destination frame exists, and save it to tf.
bool GetTf(const std::string &tf_origin, const std::string &tf_dest, tf::Transform &tf, const double duration)
{
    if (ptf_listener_->frameExists(tf_origin) && ptf_listener_->frameExists(tf_dest))
    {
        if (ptf_listener_->waitForTransform(tf_origin, tf_dest, ros::Time(0), ros::Duration(duration)))
        {
            try
            {
                tf::StampedTransform transform;
                ptf_listener_->lookupTransform(tf_origin, tf_dest, ros::Time(0), transform);
                tf = transform;
                return true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                return false;
            }
        }
        else
        {
            ROS_WARN("Transform not found");
            return false;
        }
    }
    else
    {
        ROS_WARN("One of the given frames doesn't exist");
        if(!ptf_listener_->frameExists(tf_origin))
            ROS_WARN("Origin frame -%s- doesn't exist", tf_origin.c_str());
        if(!ptf_listener_->frameExists(tf_dest))
            ROS_WARN("Destination frame -%s- doesn't exist", tf_dest.c_str());
        return false;
    }
}

//  Write the detections on file, after calulating the mean value for every tag.
void WriteDetections ()
{
    y_out_ << YAML::BeginSeq;

    //  for every different tag
    for(int i = 0; i<detected_tags_array_.size(); ++i)
    {
        //  have I ever seen this tag?
        if(detected_tags_array_[i].markers.size()>0)
        {
            geometry_msgs::Pose t;
            t.position.x = 0;
            t.position.y = 0;
            t.position.z = 0;
            t.orientation.x = 0;
            t.orientation.y = 0;
            t.orientation.z = 0;
            t.orientation.w = 0;
            //	calculating pose mean value.
            for(int j = 0; j<detected_tags_array_[i].markers.size(); ++j)
            {
                t.position.x += detected_tags_array_[i].markers[j].pose.position.x*1/detected_tags_array_[i].markers.size();
                t.position.y += detected_tags_array_[i].markers[j].pose.position.y*1/detected_tags_array_[i].markers.size();
                t.position.z += detected_tags_array_[i].markers[j].pose.position.z*1/detected_tags_array_[i].markers.size();
                t.orientation.x += detected_tags_array_[i].markers[j].pose.orientation.x*1/detected_tags_array_[i].markers.size();
                t.orientation.y += detected_tags_array_[i].markers[j].pose.orientation.y*1/detected_tags_array_[i].markers.size();
                t.orientation.z += detected_tags_array_[i].markers[j].pose.orientation.z*1/detected_tags_array_[i].markers.size();
                t.orientation.w += detected_tags_array_[i].markers[j].pose.orientation.w*1/detected_tags_array_[i].markers.size();
            }
            detected_tags_array_[i].markers[0].pose = t;
            //	renaming namespace to keep markers visible when loaded.
            detected_tags_array_[i].markers[0].ns = detected_tags_array_[i].markers[0].ns + "_old";
            y_out_ << detected_tags_array_[i].markers[0];
        }
    }
    y_out_ << YAML::EndSeq;
	std::ofstream fout(yaml_path_.c_str()); 
    fout << y_out_.c_str(); 
}

//  Load launch file parameter.
void GetParameterValues()
{
    node_->param("max_tag_id", max_id_, DEFAULT_MAX_TAG_ID);
    node_->param("yaml_file_path", yaml_path_, DEFAULT_YAML_FILE_PATH);
    node_->param("max_dist", max_dist_, DEFAULT_MAX_DIST);
    node_->param("max_angle", max_angle_, DEFAULT_MAX_ANGLE);
    //  converting degree in radians
    max_angle_ = max_angle_ * 3.14159/180;
    node_->param("map_frame", map_frame_, DEFAULT_MAP_FRAME);
    node_->param("camera_frame", camera_frame_, DEFAULT_CAMERA_FRAME);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "write_markers");
    
    node_ =  boost::make_shared<ros::NodeHandle>("~");
	ptf_listener_.reset(new tf::TransformListener());
	det_sub_ = node_->subscribe("/apriltags/marker_array", 1000, MarkerCallback);

	GetParameterValues();
	detected_tags_array_.resize(max_id_);

	ROS_INFO("WriteMarkers node started.");

    signal(SIGINT, mySigintHandler);
	ros::spin();

	return EXIT_SUCCESS;
}
