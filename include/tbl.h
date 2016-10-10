//              Default parameters values
const double DEFAULT_TRANSFORM_RATE = 10;
const double DEFAULT_START_DELAY = 2;
const double DEFAULT_DISTANCE_THRESHOLD = 1.5;
const double DEFAULT_PRECISION = 0.2;
const int DEFAULT_CORRECTION_RATE = 5;
const std::string DEFAULT_BASE_FRAME = "base_link";
const std::string DEFAULT_ODOM_FRAME = "odom";
const std::string DEFAULT_MAP_FRAME = "map";

//              Global variables

//                  parameters
double transform_rate_;
double start_delay_;
double distance_threshold_;
double precision_;
std::string base_frame_;
std::string odom_frame_;
std::string map_frame_;

//                  variables
boost::shared_ptr<tf::TransformBroadcaster> ptf_broadcaster_;
boost::shared_ptr<tf::TransformListener> ptf_listener_;
ros::NodeHandlePtr node_;
tf::Transform odom_tag_pose_;
tf::Transform odom_odom_pose_;
tf::Transform last_pose_;
geometry_msgs::TransformStamped published_base_;
tf::StampedTransform tf_map_to_odom_;
nav_msgs::Odometry odom_msg_;
ros::Subscriber odom_sub_;
ros::Subscriber estimated_pose_sub_;
ros::Time transform_expiration_;
bool first_message_;

void OdomCallback (const nav_msgs::Odometry& published_odom);
void EstimatedLocCallback (const geometry_msgs::TransformStamped& published_base);
void GetParameterValues(void);
void InitNode(int argc, char **argv);
bool GetTf(const std::string &tf_origin, const std::string &tf_dest, tf::Transform &tf, const double duration);
void OdomFromBase();
void CalculateRealOdom();
double CalculateDistance (tf::Transform, tf::Transform);
void CorrectPose ();
