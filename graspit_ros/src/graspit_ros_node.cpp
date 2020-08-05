
#include <grasp_planning_graspit/GraspItSceneManagerHeadless.h>
#include <grasp_planning_graspit/EigenGraspPlanner.h>
#include <grasp_planning_graspit/EigenGraspResult.h>
#include <graspit_ros/GraspPlanning.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

// Create the graspit world manager.
SHARED_PTR<GraspIt::GraspItSceneManager> graspitMgr(new GraspIt::GraspItSceneManagerHeadless());
// Create the planner which accesses the graspit world.
std::string name = "EigenGraspPlanner1";
SHARED_PTR<GraspIt::EigenGraspPlanner> planner(new GraspIt::EigenGraspPlanner(name, graspitMgr));

//this service load a world and run the Eigengrasp planner as a response returns the grasping poses.
bool grasp_planning(graspit_ros::GraspPlanning::Request  &req,
                    graspit_ros::GraspPlanning::Response &res)
{

    std::string worldFilename = req.world_file_path;
    std::string outputDirectory = req.output_path;

    // Number of iterations for the planning algorithm
    int maxPlanningSteps = req.maxPlanningSteps;
    // Number of times to repeat the planning process
    int repeatPlanning = req.repeatPlanning;
    // Maximum number of planning results to keep (of each planning repeat)
    int keepMaxPlanningResults = req.keepMaxPlanningResults;
    // Finalize each planning result with an "auto-grasp" to ensure there really are
    // contacts between fingers and objects (sometimes, the grasp result is just very
    // close to the object, but not really touching it).
    bool finishWithAutograsp = req.finishWithAutograsp;

    res.grasping_poses.resize(keepMaxPlanningResults*repeatPlanning);


    // Load the graspit world
    //std::cout<<"\nPRESS 1..."<<std::endl;
    //std::cin.get();
    graspitMgr->loadWorld(worldFilename);

    // if the output directory does not exist yet, create it?
    bool createDir = true;

    // in case one wants to view the initial world before planning, save it:
    graspitMgr->saveGraspItWorld(outputDirectory + "/worlds/startWorld.xml", createDir);
    graspitMgr->saveInventorWorld(outputDirectory + "/worlds/startWorld.iv", createDir);

    // By default, the last robot loaded and the last object loaded are to be used as the hand and the
    // object to grasp for the planning process. You can use the other EigenGraspPlanner::plan()
    // method with more parameters to change this.
    if (!planner->plan(maxPlanningSteps, repeatPlanning, keepMaxPlanningResults, finishWithAutograsp))
    {
        std::cerr << "Error doing the planning." << std::endl;
        return 1;
    }

    // Now, save the results as world files.

    if(req.saveVisualResults)
    {
        // specify where you want to save them:
        std::string resultsWorldDirectory = outputDirectory + "/worlds";
        // each result file will start with this prefix:
        std::string filenamePrefix = "world";
        // specify to save as inventor files:
        bool saveInventor = true;
        // specify to save as graspit world files:
        bool saveGraspit = true;

        planner->saveResultsAsWorldFiles(resultsWorldDirectory, filenamePrefix, saveGraspit, saveInventor, createDir);
    }
    // Iterate through all results and print information about the grasps:
    std::cout<<"Results are saved"<<std::endl;
    std::vector<GraspIt::EigenGraspResult> allGrasps;

    planner->getResults(allGrasps);
    std::cout << "Grasp results:" << std::endl;
    std::vector<GraspIt::EigenGraspResult>::iterator it;
    unsigned int i=0;
    std::cout<<"Grasp size" <<allGrasps.size()<<std::endl;
    for (it = allGrasps.begin(); it != allGrasps.end(); ++it)
    {
        std::vector<double>conf = (*it).getEigenGraspValues();
        Eigen::Quaterniond ori((*it).getObjectToHandTransform().rotation());
        //returns the position in meters (Graspit works in mm).
        res.grasping_poses[i].translation.x=(*it).getObjectToHandTransform().translation()(0)*0.001;
        res.grasping_poses[i].translation.y=(*it).getObjectToHandTransform().translation()(1)*0.001;
        res.grasping_poses[i].translation.z=(*it).getObjectToHandTransform().translation()(2)*0.001;
        res.grasping_poses[i].rotation.x=ori.x();
        res.grasping_poses[i].rotation.y=ori.y();
        res.grasping_poses[i].rotation.z=ori.z();
        res.grasping_poses[i].rotation.w=ori.w();
        std::cout<<"size of Eigen Vector :" <<conf.size();
        std::cout<<"Eigen Grasp Vector : { " <<conf[0]<<" , "<<conf[1]<<" , "<<conf[2]<<" , "<<conf[3]<<" , "<<conf[4]<<" }"<<std::endl;

        //std::cout << (*it).getObjectToHandTransform().rotation() << std::endl;;
        std::cout <<"Obj to hand rot: "<<ori.w()<<" "<< ori.x()<<" "<<ori.y()<<" "<<ori.z()<< std::endl;;
        std::cout <<"Obj to hand translation: "<< res.grasping_poses[i].translation.x<<" "
                  <<res.grasping_poses[i].translation.y<<" "<<res.grasping_poses[i].translation.z<< std::endl;;
        i++;

    }

    std::vector<std::string> object=graspitMgr->getObjectNames(true);
    std::vector<std::string> obstacle=graspitMgr->getObjectNames(false);
    std::vector<std::string> robot=graspitMgr->getRobotNames();
    for(int i=0; i<object.size();i++) graspitMgr->removeObject(object[i]);
    for(int i=0; i<obstacle.size();i++) graspitMgr->removeObject(obstacle[0]);
    for(int i=0; i<robot.size();i++) graspitMgr->removeRobot(robot[0]);

    std::cout<<"objects = "<<object[0]<<std::endl;
    std::cout<<"obstacle = "<<obstacle[0]<<std::endl;
    std::cout<<"robots = "<<robot[0]<<std::endl;

    //std::cout<<"\nPRESS END..."<<std::endl;
    //std::cin.get();

    //std::cout<<"Robot + object "<<grpobj.at(0)<<" "<<robot.at(0)<<std::endl;

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasping_node");
    ros::NodeHandle n;

    ros::ServiceServer service1 = n.advertiseService("graspit_ros_node/grasp_planning", grasp_planning);
    ROS_INFO("Grasp planning service advertised successfully!");
    ros::spin();
}
