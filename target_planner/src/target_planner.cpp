#include "ros/ros.h"
#include "target_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "llllll");
    ros::NodeHandle nh("target_planner");
    string frame_id = "/map";

    ROS_INFO("Start target Planner ...");
    TargetPlanner planner(nh, frame_id);
    ROS_INFO("Target Planner Started");

    ros::spin();
    return 0;
}
