/*
	This node performs localization by apriltags detection. The estimated localization is published on /estimated_loc topic as tf, and on
	/estimated_loc_marker_array as marker for RViz.
*/

//		Libraries

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vis_msgs_marker.tpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "yaml-cpp/yaml.h"
#include "estimated_localization.h"
#include <sstream>
#include <fstream>

//		Callbacks

void MarkerCallback (const visualization_msgs::MarkerArray& published_markers)
{
	//	writable copy
	markers_ = published_markers;
	// 	for every tag detected, calculate the estimated position
	CalculateLoc();
	//	Calculate mean value of the base pose
	if(base_msgs_.size() > 0)
		CalculateMeanValue();
	// else
	// {
	// 	//	Remove base marker
	// 	base_marker_.action = 2;
	// 	markers_.markers.push_back(base_marker_);
	// }
	//	if required, display the tags saved by write_markers node too.
	if (vis_real_markers_)
		markers_.markers.insert(markers_.markers.end(), real_markers_.markers.begin(), real_markers_.markers.end());
	//	Republish tags on /estimated_loc/marker_array
	marker_pub_.publish(markers_);
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

//	Load tags saved by write_markers node.
void LoadTags ()
{
	YAML::Node fy = YAML::LoadFile(yaml_path_);
	
	visualization_msgs::Marker tmp_marker;

	for (YAML::const_iterator it=fy.begin(); it!=fy.end(); ++it)
	{
		//	Converting from geometry_msgs::Pose to tf::Transform
		tmp_marker = it->as<visualization_msgs::Marker>();
		tf::Vector3 tmp_vec(tfScalar(tmp_marker.pose.position.x), tfScalar(tmp_marker.pose.position.y), tfScalar(tmp_marker.pose.position.z));
		tf::Quaternion tmp_q;
		tf::quaternionMsgToTF(tmp_marker.pose.orientation, tmp_q);
		tf::Transform tmp_tf(tmp_q, tmp_vec);
		//tf::Transform tmp_tf(tf::Quaternion::getIdentity(), tmp_vec);
		//	Fill-up vector
		tag_real_poses_.push_back(tmp_tf);
		tmp_marker.color.r = 0;
		tmp_marker.color.b = 0;
		tmp_marker.color.g = 1;
		real_markers_.markers.push_back(tmp_marker);
	}

	//	saving max_id_ to avoid false tag detection
	max_id_ = tag_real_poses_.size();	
}

//	Calculate location from detected tags
void CalculateLoc()
{
	//	for every detected tag..
	for(int i = 0; i < markers_.markers.size(); ++i)
	{
		//	Avoiding false tag detection
		if(markers_.markers[i].id > max_id_)
		{
            ROS_ERROR("Marker ID > MAX_ID = %d", max_id_);
			continue;
		}

		//	distance
		double dx = markers_.markers[i].pose.position.x;
		double dy = markers_.markers[i].pose.position.y;
		double dz = markers_.markers[i].pose.position.z;
		
		double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));

		//	Use only near tag
		if(distance > max_dist_)
		{
			ROS_ERROR("Tag distance > MAX_DIST");
			continue;
		}

		//	Checking for angles only for near tags
		double yaw = tf::getYaw(markers_.markers[i].pose.rotation);
		double pitch = tf::getPitch(markers_.markers[i].pose.rotation);
		
		if(abs(yaw) > max_angle_ || abs(pitch) > max_angle_)	
		{
			ROS_ERROR("Tag angles > MAX_DIST");
			continue;
		}

		//	Converting from geometry_msgs::Pose to tf::Transform and computing base pose	
		tf::Vector3 tmp_vec(tfScalar(markers_.markers[i].pose.position.x), tfScalar(markers_.markers[i].pose.position.y), tfScalar(markers_.markers[i].pose.position.z));
		tf::Quaternion tmp_q;
		tf::quaternionMsgToTF(markers_.markers[i].pose.orientation, tmp_q);
		tf::Transform tf_kinect2tag(tmp_q, tmp_vec);

		tf::Transform tf_base2kinect;

        //if(GetTf(base_frame_, markers_.markers[i].header.frame_id, tf_base2kinect, 0.2))
        if(GetTf(base_frame_, camera_frame_, tf_base2kinect, 0.2))
		{
			//	compute base to detected tag Transform
			tf::Transform tf_base2tag = tf_base2kinect*tf_kinect2tag;
			//	calculate base_pose as tag_pose * tf_tag2base where tag_pose has /map as reference frame and is loaded from file. 
			tf::Transform base_pose = tag_real_poses_[markers_.markers[i].id] * tf_base2tag.inverse();
            geometry_msgs::Transform base_msg;

            //	normalizing rotation
			base_pose.setRotation(base_pose.getRotation().normalize());
            //  correcting z value to 0
            tf::Vector3 tmp_vec = base_pose.getOrigin();
            tmp_vec.setZ(0);
            base_pose.setOrigin(tmp_vec);

            tf::transformTFToMsg(base_pose, base_msg);

            //	push back. We will after compute the mean value.
			base_msgs_.push_back(base_msg);
		}
	}	
}

//	compute the mean value for every calculated base pose
void CalculateMeanValue ()
{
	geometry_msgs::TransformStamped msg;
	msg.transform.translation.x = 0;
	msg.transform.translation.y = 0;
	msg.transform.translation.z = 0;
	msg.transform.rotation.x = 0;
	msg.transform.rotation.y = 0;
	msg.transform.rotation.z = 0;
	msg.transform.rotation.w = 0;
	msg.header.frame_id = map_frame_;
	msg.child_frame_id = "estimated_base";

	for(int i = 0; i < base_msgs_.size(); ++i)	
	{
		msg.transform.translation.x += base_msgs_[i].translation.x/base_msgs_.size();
		msg.transform.translation.y += base_msgs_[i].translation.y/base_msgs_.size();
		msg.transform.translation.z += base_msgs_[i].translation.z/base_msgs_.size();
		msg.transform.rotation.x += base_msgs_[i].rotation.x/base_msgs_.size();
		msg.transform.rotation.y += base_msgs_[i].rotation.y/base_msgs_.size();
		msg.transform.rotation.z += base_msgs_[i].rotation.z/base_msgs_.size();
		msg.transform.rotation.w += base_msgs_[i].rotation.w/base_msgs_.size();
	}
	//	publish the pose as geometry_msgs::Transform
	loc_pub_.publish(msg);

	//	if required visualize an additional marker for the base
	if(vis_base_marker_)
	{
		base_marker_.pose.orientation = msg.transform.rotation;
		base_marker_.pose.position.x = msg.transform.translation.x;
		base_marker_.pose.position.y = msg.transform.translation.y;
		base_marker_.pose.position.z = msg.transform.translation.z;

		markers_.markers.push_back(base_marker_);
	}

	//	clear base_poses vector
	base_msgs_.clear();
}

//	Load parameters from launch file
void GetParameterValues()
{
    node_->param("yaml_file_path", yaml_path_, DEFAULT_YAML_FILE_PATH);
    node_->param("max_dist", max_dist_, DEFAULT_MAX_DIST);
    node_->param("max_angle", max_angle_, DEFAULT_MAX_ANGLE);
    //	converting degree in radians
    max_angle_ = max_angle_ * 3.14159/180;
    node_->param("base_frame", base_frame_, DEFAULT_BASE_FRAME);
    node_->param("map_frame", map_frame_, DEFAULT_MAP_FRAME);
    node_->param("camera_frame", camera_frame_, DEFAULT_CAMERA_FRAME);
    node_->param("vis_base_marker", vis_base_marker_, false);
    node_->param("vis_real_markers", vis_real_markers_, false);
}

//	Initialise the node
void InitNode(int argc, char **argv)
{
	ros::init(argc, argv, "estimated_localization");

    node_ =  boost::make_shared<ros::NodeHandle>("~");
	//	initialise tf_listener pointer
	ptf_listener_.reset(new tf::TransformListener());
	det_sub_ = node_->subscribe("/apriltags/marker_array", 1000, MarkerCallback);

	GetParameterValues();
    LoadTags();

    //	set-up publishers
    loc_pub_ = node_->advertise<geometry_msgs::TransformStamped>("/estimated_loc", 100);
    marker_pub_ = node_->advertise<visualization_msgs::MarkerArray>("/estimated_loc_marker_array", 100);

    //	initialise some informations for the optional base marker.
	base_marker_.header.frame_id = map_frame_;
	base_marker_.ns = "estimated_base_marker";
	base_marker_.id = 1;
	base_marker_.type = 3;
	base_marker_.action = 0;
	base_marker_.lifetime = ros::Duration(0.5);
	base_marker_.scale.x = 0.4;
	base_marker_.scale.y = 0.4;
	base_marker_.scale.z = 1;
	base_marker_.color.r = 0;
	base_marker_.color.g = 1;
	base_marker_.color.b = 0;
	base_marker_.color.a = 1;
}

int main(int argc, char **argv)
{
	InitNode(argc, argv);
	ROS_INFO("Estimated Localization node started");
	
	ros::spin();

	ROS_WARN("Estimated Localization node stopped");
	return EXIT_SUCCESS;
}
