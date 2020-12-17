#include <cmath>
#include <geometry_msgs/Pose.h>
#include "gazebo_client.hpp"
#include "model_primitives.hpp"

class CityBuilder
{
private:
    GazeboClient model_spawner;
    static constexpr double DBG_SCALE=0.04;

public:
    CityBuilder(const ros::NodeHandle &_nh) : model_spawner(_nh) {}
    ~CityBuilder() {}
    static bool assertNormal(const double &val)
    {
        return val > 0 && val <= 1;
    }
    static double denormalize(const double &normalizedVal, const double &minVal, const double &maxVal)
    {
        return minVal + normalizedVal * (maxVal - minVal);
    }

    static double dbgScale(const double val, bool is_area = false, const double scale_factor = DBG_SCALE)
    {
        return is_area ?  val * scale_factor * scale_factor: val * scale_factor;
    }
    /**
     * @brief spawn building with dimensions defined relative(0-1) of city distribution
     * 
     * @param rel_floors 
     * @param rel_floor_area 
     */
    void spawnBuilding(
        const double rel_floor_area,
        const double rel_height,
        const double pos_x,
        const double pos_y,
        const std::string name = "foo_tower")
    {
        /*
        Average height of a floor is 3.65m, with floors 3 to 15.
        Mehgrath area: 487 -> 22m
        cellestial Dreams: 1361 -> 37m

        ref: https://www.emporis.com/statistics/tallest-buildings/city/100252/surat-india
        https://housing.com/in/buy/projects/page/25590-megh-rath-by-megh-mayur-builders-organisers-in-athwa
        */

        assert(assertNormal(rel_height) && assertNormal(rel_floor_area));
        const int MIN_FLOORS = 3, MAX_FLOORS = 15;
        const double FLOOR_HEIGHT = dbgScale(3.65), MIN_HEIGHT = MIN_FLOORS * FLOOR_HEIGHT, MAX_HEIGHT = MAX_FLOORS * FLOOR_HEIGHT;
        const double MIN_FLOOR_AREA = dbgScale(240, true), MAX_FLOOR_AREA = dbgScale(1360, true);

        double building_height = denormalize(rel_height, MIN_HEIGHT, MAX_HEIGHT);
        double floor_area = denormalize(rel_floor_area, MIN_FLOOR_AREA, MAX_FLOOR_AREA);
        double floor_len = sqrt(floor_area);

        geometry_msgs::Pose pose;
        pose.position.x = pos_x;
        pose.position.y = pos_y;
        pose.position.z = building_height / 2;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        auto building_SDF = ModelPrimitives::getBox(floor_len, building_height, name);
        model_spawner.spawnModel(building_SDF, pose, name);
    }
    void spawnCity()
    {
        const double MAX_CITY_X = dbgScale(250), MAX_CITY_Y = MAX_CITY_X;
        spawnBuilding(0.01, 0.5, 0., 0., "origin_tower");
        spawnBuilding(0.2, 0.99, MAX_CITY_X * 0.5, MAX_CITY_Y * 0.5, "dest_tower");
    }
};