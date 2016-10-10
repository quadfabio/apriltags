#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>

namespace YAML
{
    Emitter& operator << (Emitter& y_out_, const visualization_msgs::Marker marker) 
    {
        y_out_ << BeginMap;
        y_out_ << Key << "frame_id"; y_out_ << Value << marker.header.frame_id;
        y_out_ << Key << "ns"; y_out_ << Value << marker.ns;
        y_out_ << Key << "id"; y_out_ << Value << marker.id;
        y_out_ << Key << "type"; y_out_ << Value << marker.type;
        y_out_ << Key << "action"; y_out_ << Value << marker.action;
        y_out_ << Key << "pose"; 
        y_out_ << Value << 
        BeginMap << 
            Key  << "position" << Value <<
            BeginMap << 
                Key << "x" << Value << marker.pose.position.x <<
                Key << "y" << Value << marker.pose.position.y <<
                Key << "z" << Value << marker.pose.position.z <<
            EndMap   <<
            Key  << "orientation" << Value <<
            BeginMap <<
                Key << "x" << Value << marker.pose.orientation.x <<
                Key << "y" << Value << marker.pose.orientation.y <<
                Key << "z" << Value << marker.pose.orientation.z <<
                Key << "w" << Value << marker.pose.orientation.w <<
            EndMap <<
        EndMap;                    
        y_out_ << Key << "scale"; 
        y_out_ << Value << 
        BeginMap <<
            Key << "x" << Value << marker.scale.x <<
            Key << "y" << Value << marker.scale.y <<
            Key << "z" << Value << marker.scale.z <<
        EndMap;
        y_out_ << Key << "color"; 
        y_out_ << Value << 
        BeginMap <<
            Key << "r" << Value << marker.color.r <<
            Key << "g" << Value << marker.color.g <<
            Key << "b" << Value << marker.color.b <<
            Key << "a" << Value << marker.color.a <<
        EndMap;
        y_out_ << EndMap;
        return y_out_;
    }

    template<>
    
    struct  convert<visualization_msgs::Marker>
    {
        static bool decode(const Node& node, visualization_msgs::Marker& var)
        {
            var.header.frame_id = node["frame_id"].as<std::string>();
            var.ns = node["ns"].as<std::string>();
            var.id = node["id"].as<int>();
            var.type = node["type"].as<int>();
            var.action = node["action"].as<int>();
            const Node& pose = node["pose"];
            const Node& position = pose["position"];
            const Node& orientation = pose["orientation"];
            var.pose.position.x = position["x"].as<double>();
            var.pose.position.y = position["y"].as<double>();
            var.pose.position.z = position["z"].as<double>();
            var.pose.orientation.x = orientation["x"].as<double>();
            var.pose.orientation.y = orientation["y"].as<double>();
            var.pose.orientation.z = orientation["z"].as<double>();
            var.pose.orientation.w = orientation["w"].as<double>();
            const Node& scale = node["scale"];
            var.scale.x = scale["x"].as<double>();
            var.scale.y = scale["y"].as<double>();
            var.scale.z = scale["z"].as<double>();
            const Node& color = node["color"];
            var.color.r = color["r"].as<double>();
            var.color.g = color["g"].as<double>();
            var.color.b = color["b"].as<double>();
            var.color.a = color["a"].as<double>();
            return true;
        }
    };
}