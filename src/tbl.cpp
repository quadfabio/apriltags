/*
    This node localize the robot during the navigation in a known map with known tags position. If it finds a (big) difference between the pose calculate from odom informations and the one calculated from tag detetcion, it 
    will correct it.
*/

//        Libraries

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "tbl.h"

//      Callbacks

void OdomCallback (const nav_msgs::Odometry& published_odom)
{
    //  writable copy
    odom_msg_ = published_odom;
    //  have I ever received a message from /odom?
    first_message_ = true;
}

void EstimatedLocCallback (const geometry_msgs::TransformStamped& published_base)
{
    //  writable copy
    published_base_ = published_base;
    //  compute odom pose from base pose published by estimated_localization node
    OdomFromBase();
}

//      Functions

//  load parameters from launch file
void GetParameterValues()
{
    node_->param("rate", transform_rate_, DEFAULT_TRANSFORM_RATE);
    node_->param("start_delay", start_delay_, DEFAULT_START_DELAY);
    node_->param("distance_threshold", distance_threshold_, DEFAULT_DISTANCE_THRESHOLD);
    node_->param("precision", precision_, DEFAULT_PRECISION);
    node_->param("base_frame", base_frame_, DEFAULT_BASE_FRAME);
    node_->param("odom_frame", odom_frame_, DEFAULT_ODOM_FRAME);
    node_->param("map_frame", map_frame_, DEFAULT_MAP_FRAME);
}

//  initialise the node
void InitNode(int argc, char **argv)
{
    ros::init(argc, argv, "tbl");
    
    node_ = boost::make_shared<ros::NodeHandle>("~");
    //  initialise tf_listener pointer
    ptf_listener_.reset(new tf::TransformListener());
    //  initialise tf_broadcaster pointer
    ptf_broadcaster_.reset(new tf::TransformBroadcaster());
    
    //  subscribers
    odom_sub_ = node_->subscribe("/odom", 100, OdomCallback);
    estimated_pose_sub_ = node_->subscribe("/estimated_loc", 100, EstimatedLocCallback);

    //  initialise last_pose_ object, to avoid problems when comparing it to odom_tag_pose_ for the first time.
    last_pose_ = tf::Transform::getIdentity();
    GetParameterValues();
    //  intialise the variable. I haven't receveid a message from /odom yet.
    first_message_ = false;
}

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

//  compute odom pose from base pose 
void OdomFromBase()
{
    tf::Transform tf_base2odom;
    tf::Transform tf_base;
    tf::transformMsgToTF(published_base_.transform, tf_base);

    if(GetTf(base_frame_, odom_frame_, tf_base2odom, 0.2))
        odom_tag_pose_ = tf_base * tf_base2odom;
}

//  check for a big difference between odom pose given by odom informations and the one given by tag detection
void CalculateRealOdom()
{
    if (CalculateDistance(odom_odom_pose_, odom_tag_pose_) < distance_threshold_)
    {
        //  odom pose given by odom informations is correct, just publishing the transform as given by the robot
        tf_map_to_odom_ = tf::StampedTransform(odom_odom_pose_, transform_expiration_, std::string(map_frame_), std::string(odom_frame_));
    }
    else
    {   
        //  tag detection as a precision of some centimeters. Let's change the pose if we notice a great change. This avoid robot oscillation on RViz
        if(CalculateDistance(odom_tag_pose_, last_pose_) > precision_)
        {
            //  We only care about position.x, position.y and yaw. In this way we avoid uncorrect orientation due to kinect oscillation.
            CorrectPose();
            last_pose_ = odom_tag_pose_;
        }
        else
        {
            //  robot hasn't moved
            odom_tag_pose_ = last_pose_;
        }
        tf_map_to_odom_ = tf::StampedTransform(odom_tag_pose_, transform_expiration_, std::string(map_frame_), std::string(odom_frame_));
    }
}

//  calculate 2D distance between two tf::Transform, treated as poses
double CalculateDistance (tf::Transform tf_a, tf::Transform tf_b)
{
    double dx = tf_a.getOrigin().x() - tf_b.getOrigin().x();
    double dy = tf_a.getOrigin().y() - tf_b.getOrigin().y();
    return sqrt(pow(dx, 2)+pow(dy, 2));
}

//  extrapolating only informations we care about assuming that turtles can't fly.
void CorrectPose()
{
    //  Rotation
    tf::Matrix3x3 tmp_mat(odom_tag_pose_.getRotation());
    tfScalar yaw, pitch, roll;
    tmp_mat.getEulerYPR(yaw, pitch, roll);
    tf::Quaternion tmp_q;
    //  setEulerZYX is deprecated
    //tmp_q.setEulerZYX(yaw, 0.0, 0.0);
    tmp_q.setEuler(0.0, 0.0, yaw);
    tmp_q.normalize();

    odom_tag_pose_.setRotation(tmp_q);

    //  Position
    tf::Vector3 tmp_v(odom_tag_pose_.getOrigin());
    tmp_v.setZ(0.0);

    odom_tag_pose_.setOrigin(tmp_v);
}

int main(int argc, char **argv)
{
    InitNode(argc, argv);
    ROS_INFO("Tag Based Localization node started");

    ros::Rate loop_rate(transform_rate_);

    ros::Time start = ros::Time::now();

    //  wait for first odom message
    while(!first_message_)
    {
        ROS_WARN("Publishing map tf. Waiting for /odom first message");
        transform_expiration_ = ros::Time::now() + ros::Duration(1/transform_rate_);
        tf_map_to_odom_ = tf::StampedTransform(tf::Transform::getIdentity(), transform_expiration_, std::string(map_frame_), std::string(odom_frame_));

        ptf_broadcaster_->sendTransform(tf_map_to_odom_);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //  main loop
    while (ros::ok())
    {
        transform_expiration_ = odom_msg_.header.stamp + ros::Duration(1/transform_rate_);

        //  wait for a given delay before start
        if((ros::Time::now()-start) < ros::Duration(start_delay_))
        {
            ROS_WARN("Start delay");
            tf_map_to_odom_ = tf::StampedTransform(tf::Transform::getIdentity(), transform_expiration_, std::string(map_frame_), std::string(odom_frame_));
        }
        else
        {
            tf::Vector3 tmp_pos(odom_msg_.pose.pose.position.x, odom_msg_.pose.pose.position.y, odom_msg_.pose.pose.position.z);
            tf::Quaternion tmp_orient(odom_msg_.pose.pose.orientation.x, odom_msg_.pose.pose.orientation.y, odom_msg_.pose.pose.orientation.z, odom_msg_.pose.pose.orientation.w);
            odom_odom_pose_ = tf::Transform(tmp_orient, tmp_pos);

            CalculateRealOdom();
        }

        //  broadcast transform
        ptf_broadcaster_->sendTransform(tf_map_to_odom_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
