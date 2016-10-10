//		Default parameters values

const std::string DEFAULT_YAML_FILE_PATH = "/home/fabio/detected_tag.yaml";
const double DEFAULT_MAX_DIST = 25;
const double DEFAULT_MAX_OBL_DIST = 15;
const std::string DEFAULT_BASE_FRAME = "base_link";
const std::string DEFAULT_MAP_FRAME = "map";
const std::string DEFAULT_CAMERA_FRAME = "kinect2_head_rgb_optical_frame";


//		Global variables

//			paramaters
std::string yaml_path_;	
std::string map_frame_;
std::string base_frame_;
std::string camera_frame_;
double max_dist_;
double max_obl_dist_;
bool vis_base_marker_;
bool vis_real_markers_;

//			variables
int max_id_;
visualization_msgs::MarkerArray markers_;
visualization_msgs::MarkerArray real_markers_;
visualization_msgs::Marker base_marker_;
std::vector<tf::Transform> tag_real_poses_;
// std::vector<tf::Vector3> tag_real_poses_;
std::vector<geometry_msgs::Transform> base_msgs_;
boost::shared_ptr<tf::TransformListener> ptf_listener_;
ros::NodeHandlePtr node_;
ros::Subscriber det_sub_;
ros::Publisher loc_pub_;
ros::Publisher marker_pub_;

void MarkerCallback (const visualization_msgs::MarkerArray& published_markers);
bool GetTf(const std::string &tf_origin, const std::string &tf_dest, tf::Transform &tf, const double duration);
void LoadTags (void);
void CalculateLoc (void);
void CalculateMeanValue(void);
void GetParameterValues(void);
void InitNode(int argc, char **argv);
