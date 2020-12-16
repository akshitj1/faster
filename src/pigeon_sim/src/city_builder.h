#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <geometry_msgs/Pose.h>
#include <ignition/math.hh>
#include <tinyxml2.h>

/**
 * @brief Utility class to get sdf tree for gazebo model primitives like ground_plane, cube, cylinders
 * 
 */
class ModelPrimitives
{
private:
    /* data */
public:
    ModelPrimitives() {}
    ~ModelPrimitives() {}

    static std::string xmlDocToString(const tinyxml2::XMLDocument& doc){
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        std::string doc_str(printer.CStr());
        return doc_str;
    }

    static std::string getCube(const double mass = 1.0, const double edge_length = 1.0){
        tinyxml2::XMLDocument doc;
        auto sdf = doc.NewElement("sdf");
        doc.InsertFirstChild(sdf);
        sdf->SetAttribute("version", "1.4");
        auto cube = sdf->InsertNewChildElement("model");
        cube->SetAttribute("name", "foo_cube");

        ignition::math::Pose3d pose(5., 5., 5., 0., 0., 0.);
        std::stringstream pose_str;
        pose_str << pose;
        cube->InsertNewChildElement("pose")->SetText(pose_str.str().c_str());
        cube->InsertNewChildElement("static")->SetText(true);
        auto link = cube->InsertNewChildElement("link");
        link->SetAttribute("name", "link");

        // calculate inertia
        const double xx = mass / 12 * (edge_length * edge_length + edge_length * edge_length), yy = xx, zz = xx;
        auto inertial = link->InsertNewChildElement("inertial");
        inertial->InsertNewChildElement("mass")->SetText(mass);
        auto inertia = inertial->InsertNewChildElement("inertia");
        inertia->InsertNewChildElement("ixx")->SetText(xx);
        inertia->InsertNewChildElement("ixy")->SetText(0.0);
        inertia->InsertNewChildElement("ixz")->SetText(0.0);
        inertia->InsertNewChildElement("iyy")->SetText(yy);
        inertia->InsertNewChildElement("iyz")->SetText(0.0);
        inertia->InsertNewChildElement("izz")->SetText(zz);
        
        ignition::math::Vector3d box_size(edge_length, edge_length, edge_length);
        std::stringstream box_size_str;
        box_size_str << box_size;

        auto collision = link->InsertNewChildElement("collision");
        collision->SetAttribute("name", "collision");
        auto collision_geometry = collision->InsertNewChildElement("geometry");
        collision_geometry->InsertNewChildElement("box")->InsertNewChildElement("size")->SetText(box_size_str.str().c_str());
        // collision_geometry->AddElement("surface")->AddElement("contact")->AddElement("collide_without_contact")->AddValue("bool", "true", false);

        auto visual = link->InsertNewChildElement("visual");
        visual->SetAttribute("name", "visual");
        auto visual_geometry = visual->InsertNewChildElement("geometry");
        visual_geometry->InsertNewChildElement("box")->InsertNewChildElement("size")->SetText(box_size_str.str().c_str());

        auto material = visual->InsertNewChildElement("material");
        auto material_script = material->InsertNewChildElement("script");
        material_script->InsertNewChildElement("uri")->SetText("file://media/materials/scripts/gazebo.material");
        material_script->InsertNewChildElement("name")->SetText("Gazebo/Blue");

        link->InsertNewChildElement("self_collide")->SetText(0);
        link->InsertNewChildElement("kinematic")->SetText(0);
        link->InsertNewChildElement("gravity")->SetText(0);

        ROS_INFO_STREAM(xmlDocToString(doc));
        return xmlDocToString(doc);
    }
};

#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

class ModelSpawner
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient spawn_service;

public:
    ModelSpawner(ros::NodeHandle &_nh) : nh(_nh)
    {
        spawn_service = nh.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
    }
    ~ModelSpawner() {}

    void spawnModel(const std::string model_name, const std::string& model_sdf)
    {
        const char *model_xml = R"(
            <?xml version='1.0'?>
            <sdf version='1.4'>
            <model name='my_model'>
                <pose>0 0 0.5 0 0 0</pose>
                <static>true</static>
                <link name='link'>
                <inertial>
                    <mass>1.0</mass>
                    <inertia> <!-- inertias are tricky to compute -->
                    <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
                    <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
                    <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
                    <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
                    <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
                    <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
                    <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
                    </inertia>
                </inertial>
                <collision name='collision'>
                    <geometry>
                    <box>
                        <size>1 1 1</size>
                    </box>
                    </geometry>
                </collision>
                <visual name='visual'>
                    <geometry>
                    <box>
                        <size>1 1 1</size>
                    </box>
                    </geometry>
                </visual>
                </link>
            </model>
            </sdf>)";

        gazebo_msgs::SpawnModel spawn_call;
        spawn_call.request.model_name = model_name;
        spawn_call.request.model_xml = model_sdf;
        spawn_call.request.robot_namespace = "cube_spawner";
        geometry_msgs::Pose pose;
        pose.position.x = 5.;
        pose.position.y = 5.;
        pose.position.z = 5.;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        spawn_call.request.initial_pose = pose;
        spawn_call.request.reference_frame = "world";

        spawn_service.waitForExistence();
        if (!spawn_service.call(spawn_call))
            ROS_ERROR("Failed to call service %s", SPAWN_OBJECT_TOPIC);

        ROS_INFO("Result: %s, code %u", spawn_call.response.status_message.c_str(), spawn_call.response.success);
    }
};

class CityBuilder
{
private:
    ModelSpawner model_spawner;

public:
    CityBuilder(ros::NodeHandle &_nh) : model_spawner(_nh) {}
    ~CityBuilder() {}
    void buildCity()
    {
        auto cube_sdf = ModelPrimitives::getCube();
        // ROS_INFO("%s", cube_sdf->ToString().c_str());
        model_spawner.spawnModel("foo_cube", cube_sdf);
    }
};