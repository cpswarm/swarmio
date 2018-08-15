#include <ros/ros.h>
#include <swarmros/example/Node.h>

/**
 * Entry point for the node
 */
int main(int argc, const char* argv[])
{
    // Initialize ROS
    ros::init(argc, (char**)argv, "example");

    // Create node
    swarmros::example::Node node("example");

    // Run node
    ros::spin();

    // Exit
    return 0;
}