//		Default parameters values

const std::string DEFAULT_YAML_FILE_PATH = "/home/fabio/detected_tag.yaml";
const double DEFAULT_MAX_DIST = 25;
const double DEFAULT_MAX_ANGLE = 30;
const int DEFAULT_MAX_TAG_ID = 10;
const std::string DEFAULT_MAP_FRAME = "map";
const std::string DEFAULT_CAMERA_FRAME = "kinect2_head_rgb_optical_frame";

//		Global variables

//			paramaters
std::string yaml_path_;
std::string map_frame_;
std::string camera_frame_;
double max_dist_;
double max_angle_;
int max_id_;

//			variables
YAML::Emitter y_out_;
std::vector<visualization_msgs::MarkerArray> detected_tags_array_;
boost::shared_ptr<tf::TransformListener> ptf_listener_;
ros::NodeHandlePtr node_;
ros::Subscriber det_sub_;

void MarkerCallback (const visualization_msgs::MarkerArray& published_markers);
bool GetTf(const std::string &tf_origin, const std::string &tf_dest, tf::Transform &tf, const double duration);
void WriteDetections (void);
void GetParameterValues(void);

