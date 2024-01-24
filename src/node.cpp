#include <mrflocalization_detector/mrf.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mrf_detector");
  MRF node;
  ros::Rate loopRate(5);
  while (ros::ok()) {
    ros::spinOnce();
    node.predictFailureProbability();
    loopRate.sleep();
  }

  return 0;
}
