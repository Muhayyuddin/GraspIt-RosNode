#include <ros/ros.h>
#include <graspit_ros/GraspPlanning.h>
#include <geometry_msgs/Transform.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasping_node_client");
    ros::NodeHandle n;

    ros::ServiceClient GraspPlanning_client = n.serviceClient<graspit_ros::GraspPlanning>("graspit_ros_node/grasp_planning");
    graspit_ros::GraspPlanning GraspPlanning_srv;

    GraspPlanning_srv.request.world_file_path="/home/muhayyuddin/graspit_ws/src/GraspIt-RosNode/graspit_ros/models/worlds/Kitchen_pal_gripper.xml";
    GraspPlanning_srv.request.output_path="/home/muhayyuddin/graspit_ws/src/GraspIt-RosNode/graspit_ros/RESULTS";
    GraspPlanning_srv.request.maxPlanningSteps=70000;
    GraspPlanning_srv.request.repeatPlanning=1;
    GraspPlanning_srv.request.keepMaxPlanningResults=2;
    GraspPlanning_srv.request.finishWithAutograsp=false;
    GraspPlanning_srv.request.saveVisualResults=true;
    //GraspPlanning_srv.request.objectName="cup";
    //GraspPlanning_srv.request.actionType="serve";
    GraspPlanning_client.call(GraspPlanning_srv);
    ROS_INFO("Grasp planning service advertised successfully!");
    ros::spin();
}
