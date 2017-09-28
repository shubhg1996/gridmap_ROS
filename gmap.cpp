#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_worker");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"occmap"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03, Position(0.6, -2.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  map["occmap"].setConstant(-1.0);

  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    
    // image submap (iterators).
    Position mappos = map.getPosition();
    Position topLeftCorner(mappos.x()-0.1, mappos.y()+0.1);  \\image exploration region 
    boundPositionToRange(topLeftCorner, map.getLength(), mappos);
    Index startIndex;
    map.getIndex(topLeftCorner, startIndex);
    ROS_INFO_ONCE("Top left corner was limited from (-0.1,0.1) to (%f, %f) and corresponds to index (%i, %i).",
             topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

    Size size = (Length(0.1, 0.1) / map.getResolution()).cast<int>();
    SubmapIterator it(map, startIndex, size);
    for (; !it.isPastEnd(); ++it) {
      Position currentPosition;
      map.getPosition(*it, currentPosition);
      float objectness = 0;
      //convert to camera pixels and extract region
      //calculate objectness measure 
      map.at("occmap", *it) = objectness;
    }
    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    rate.sleep();
  }

  return 0;
}