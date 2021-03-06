#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include "classify/classify.h"
#include <cv_bridge/cv_bridge.h>

using namespace grid_map;
float cammat[3][3] = {{1.0, 0.0, 1.0},{0.0, 1.0, 1.0},{0.0, 0.0, 1.0}}; 

struct Retclass {
  int type;
  float prob;
};

// Initialize node and publisher.
ros::init(argc, argv, "grid_map_worker");
ros::NodeHandle nh("~");
ros::ServiceClient client = n.serviceClient<classify::classify>("classify");
imageSubscriber_ = nodeHandle_.subscribe("/image", 1, &imageCallback, this);
ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);


int main(int argc, char** argv)
{
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
    rate.sleep();
    ros::Time time = ros::Time::now();
    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
  }
  return 0;
}

//call python service for classifying
Retclass cluster(cv::Mat roi) {
  classify::classify srv;
  cv_bridge::CvImagePtr roi_ptr;
  roi_ptr->image = roi;
  srv.request.roi = roi_ptr->toImageMsg();
  client.call(srv);
  return {srv.response.class,srv.response.prob};
}

//project imagepoint and objectness calculation
Retclass findObj(Position imgpos, std::vector<Point3f> tvec, std::vector<Point3f> rvec, cv::Mat img) {
  std::vector<Point3f> imgpt;
  std::vector<Point2f> imgout;
  imgpt.push_back(Point3f(imgpos.x(),imgpos.y(),0.0));
  //project points on image and extract ROI
  cv::projectPoints(imgpt,rvec,tvec,cammat,NULL,imgout);
  cv::Rect region_of_interest = Rect(imgout[0].x-5, imgout[0].y-5, 10, 10);
  cv::Mat image_roi = img(region_of_interest);
  //get cluster class
  return cluster(image_roi);
}


//Image callback for gridmacp generation
void imageCallback(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  // image submap (iterators).
  //get rvec, tvec
  //update x, y, theta from camera pose
  Position topLeftCorner(x-0.1*(std::cos(theta)+std::sin(theta)), y+0.1*(std::cos(theta)-std::sin(theta)));
  Position topRightCorner(x+0.1*(std::cos(theta)-std::sin(theta)), y+0.1*(std::cos(theta)+std::sin(theta)));
  Position center(x, y);
  Index startIndex,stopIndex,centerIndex,currentIndex;
  map.getIndex(topLeftCorner, startIndex);
  map.getIndex(topRightCorner, stopIndex);
  map.getIndex(center, centerIndex);
  for (grid_map::LineIterator it(map, startIndex, stopIndex); !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    map.getIndex(currentPosition, currentIndex);
    for (grid_map::LineIterator it2(map, centerIndex, currentIndex); !it2.isPastEnd(); ++it2) {
      curobj = map.at("occmap", *it2);
      if (curobj == -1) {
        Position imgpt;
        map.getPosition(*it2, imgpt);
        //convert to camera pixels and extract region
        //calculate objectness measure
        Retclass out;
        out = findObj(imgpt,rvec,tvec,cv_ptr->image);
        curobj = out.type;
        //mark if lane
        if (curobj == 1) 
          map.at("occmap", *it2) = curobj;
      }
      //if ground not found
      if (curobj != 0) {
        break;
      }
    }
  }
  
}