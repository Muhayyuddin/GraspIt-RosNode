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
    if(argc<3)
    {
        std::cout << "Num of parameters is not correct\n";
        std::cout << "Should be: rosrun graspit_ros world_file_path output_path\n";

        return -1;
    }
    ROS_INFO("Starting grasping_node_client");    
    
    ros::init(argc, argv, "grasping_node_client");
    ros::NodeHandle n;
    
    ROS_INFO("Using world_file_path: %s",argv[1]);
    ROS_INFO("Using output_path: %s", argv[2]);

    ros::ServiceClient GraspPlanning_client = n.serviceClient<graspit_ros::GraspPlanning>("graspit_ros_node/grasp_planning");
    graspit_ros::GraspPlanning GraspPlanning_srv;

    GraspPlanning_srv.request.world_file_path=argv[1];
    GraspPlanning_srv.request.output_path=argv[2];
    GraspPlanning_srv.request.maxPlanningSteps=70000;
    GraspPlanning_srv.request.repeatPlanning=1;
    GraspPlanning_srv.request.keepMaxPlanningResults=2;
    GraspPlanning_srv.request.finishWithAutograsp=false;
    GraspPlanning_srv.request.saveVisualResults=true;
    //GraspPlanning_srv.request.objectName="cup";
    //GraspPlanning_srv.request.actionType="serve";
    
    
    ros::service::waitForService("graspit_ros_node/grasp_planning");
    
    GraspPlanning_client.call(GraspPlanning_srv);
    ROS_INFO("Grasp planning service called successfully!");

    ros::spin();
}

