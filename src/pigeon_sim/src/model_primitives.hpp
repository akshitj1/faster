#include <ros/console.h>
#include <tinyxml2.h>
#include <ignition/math.hh>


/**
 * @brief Utility class to get sdf tree for gazebo model primitives like ground_plane, box, cylinders
 * 
 */
class ModelPrimitives
{
private:
    /* data */
public:
    ModelPrimitives() {}
    ~ModelPrimitives() {}

    static std::string xmlDocToString(const tinyxml2::XMLDocument &doc)
    {
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        std::string doc_str(printer.CStr());
        return doc_str;
    }

    static std::string getBox(const double lx, const double lz, const std::string name = "foo_box")
    {
        tinyxml2::XMLDocument doc;
        auto sdf = doc.NewElement("sdf");
        doc.InsertFirstChild(sdf);
        sdf->SetAttribute("version", "1.4");
        auto box = sdf->InsertNewChildElement("model");
        box->SetAttribute("name", name.c_str());

        // ignition::math::Pose3d pose(5., 5., 5., 0., 0., 0.);
        // std::stringstream pose_str;
        // pose_str << pose;
        // box->InsertNewChildElement("pose")->SetText(pose_str.str().c_str());
        box->InsertNewChildElement("static")->SetText(true);
        auto link = box->InsertNewChildElement("link");
        link->SetAttribute("name", "link");

        // calculate inertia
        // since static, mass and inertial is irrelevant
        const double mass = 1.0;
        const double xx = mass / 12 * (lx * lx + lx * lx), yy = xx, zz = xx;
        auto inertial = link->InsertNewChildElement("inertial");
        inertial->InsertNewChildElement("mass")->SetText(mass);
        auto inertia = inertial->InsertNewChildElement("inertia");
        inertia->InsertNewChildElement("ixx")->SetText(xx);
        inertia->InsertNewChildElement("ixy")->SetText(0.0);
        inertia->InsertNewChildElement("ixz")->SetText(0.0);
        inertia->InsertNewChildElement("iyy")->SetText(yy);
        inertia->InsertNewChildElement("iyz")->SetText(0.0);
        inertia->InsertNewChildElement("izz")->SetText(zz);

        ignition::math::Vector3d box_size(lx, lx, lz);
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