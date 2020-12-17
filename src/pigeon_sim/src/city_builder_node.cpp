// https://github.com/osrf/sdformat/blob/sdformat6_6.2.0/examples/dom.cc
#include <ros/ros.h>

#include "city_builder.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "city_builder");
  ros::NodeHandle nh("");
  CityBuilder city_builder(nh);
  city_builder.spawnCity();
  ros::spinOnce();
  return 0;
}