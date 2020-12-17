#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>


#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"
#define DELETE_OBJECT_TOPIC "gazebo/delete_model"

class GazeboClient
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient spawn_service;
    ros::ServiceClient delete_service;

public:
    GazeboClient(const ros::NodeHandle &_nh) : nh(_nh)
    {
        spawn_service = nh.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
        delete_service = nh.serviceClient<gazebo_msgs::DeleteModel>(DELETE_OBJECT_TOPIC);
    }
    ~GazeboClient() {}

    void deleteModel(const std::string model_name)
    {
        gazebo_msgs::DeleteModel delete_model_call;
        delete_model_call.request.model_name = model_name;
        ROS_INFO("started waiting for delete service");
        delete_service.waitForExistence();
        ROS_WARN("done waiting for delete service");
        if(!delete_service.call(delete_model_call))
            ROS_ERROR("Failed to call service %s", DELETE_OBJECT_TOPIC);
        ROS_WARN("Result: %s, code %u", delete_model_call.response.status_message.c_str(), delete_model_call.response.success);

    }

    void spawnModel(const std::string &model_sdf, const geometry_msgs::Pose &pose, const std::string model_name)
    {
        deleteModel(model_name);
        gazebo_msgs::SpawnModel spawn_call;
        spawn_call.request.model_name = model_name;
        spawn_call.request.model_xml = model_sdf;
        spawn_call.request.robot_namespace = "box_spawner";
        spawn_call.request.initial_pose = pose;
        spawn_call.request.reference_frame = "world";

        spawn_service.waitForExistence();
        if (!spawn_service.call(spawn_call))
            ROS_ERROR("Failed to call service %s", SPAWN_OBJECT_TOPIC);

        ROS_WARN("Result: %s, code %u", spawn_call.response.status_message.c_str(), spawn_call.response.success);
    }
};