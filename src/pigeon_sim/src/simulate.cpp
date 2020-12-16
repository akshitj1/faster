// https://github.com/osrf/sdformat/blob/sdformat6_6.2.0/examples/dom.cc
#include <ros/ros.h>

#include "city_builder.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voxblox");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  CityBuilder city_builder(nh);
  city_builder.buildCity();
  ros::spinOnce();

  // Read an SDF file, and store the result in sdf.
  // auto sdf = sdf::readFile(argv[1]);
  return 0;
}