#include <ros/ros.h>
#include "exploration.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "srsd_planner");
    ros::NodeHandle n("~");
    ROS_INFO("srsd_planner, Start~ :)");
    srsd_planner planner(n);

    // signal(SIGINT, signal_handler);
    ros::AsyncSpinner spinner(5); // Use 4 threads -> 4 callbacks
    spinner.start();
    ros::waitForShutdown();


    return 0;
}