#include <ros/ros.h>
#include "octomap_only.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_only");
    ros::NodeHandle n("~");
    ROS_INFO("octomap_only, Start~ :)");
    octomap_only octomap_only(n);

    // signal(SIGINT, signal_handler);
    ros::AsyncSpinner spinner(5); // Use 4 threads -> 4 callbacks
    spinner.start();
    ros::waitForShutdown();


    return 0;
}