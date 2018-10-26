#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>


geometry_msgs::Pose getPose(const double position[3], const double orientation[4]) {
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.w = orientation[0];
    pose.orientation.x = orientation[1];
    pose.orientation.y = orientation[2];
    pose.orientation.z = orientation[3];
    return pose;
}

std_msgs::ColorRGBA getColor(const float colorRGBA[4]){
    std_msgs::ColorRGBA color;
    color.r = colorRGBA[0];
    color.g = colorRGBA[1];
    color.b = colorRGBA[2];
    color.a = colorRGBA[3];
    return color;
}

void add_marker(visualization_msgs::MarkerArray& marker_array,const std::string& modelName, geometry_msgs::Pose pose, std_msgs::ColorRGBA color, float scale, const std::string& ns){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = (int)marker_array.markers.size();
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    std::string meshFilePath = "package://thesis/meshes/" + modelName + ".stl";
    marker.mesh_resource = meshFilePath;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;
    marker_array.markers.push_back(marker);
}


int main(int argc, char * argv[])
{
    ROS_INFO("launching node");
    ros::init(argc, argv, "vis_pub");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(1);
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "/recognized_object";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://thesis/meshes/lf064-05.stl";
    marker.action = visualization_msgs::Marker::ADD;

    double position[3] = {1,1,1};
    double orientation[4] = {0,0,0,1};
    float color[4]={0,1,0,0.7};
    add_marker(markerArray, "lf064-05", getPose(position,orientation), getColor(color), 0.001, "/recognized_obj");

    double position2[3] = {1,0,1};
    double orientation2[4] = {0,0,0,1};
    float color2[4]={0,0,1,0.7};
    add_marker(markerArray, "lf064-04", getPose(position2,orientation2), getColor(color2), 0.001, "/recognized_obj");


    int i = 0;
    while(ros::ok()){
    vis_pub.publish( markerArray);
    ros::spinOnce();
    }
}