#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <iostream>
#include <fstream>
#include <time.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_check_benchmark_cpp");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Create and open a csv file
	std::ifstream joints_list("./two_arms_benchmarks/joints_list/joints_list.csv");
	
	std::ofstream output("./two_arms_benchmarks/results/cpp_benchmark.csv");

	robot_model_loader::RobotModelLoader robot_model_loader("/two_arms/robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup *joint_model_group_dual = kinematic_model->getJointModelGroup("dual_arm");

	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group_dual, joint_values);

	std::stringstream csv_content;

	collision_detection::CollisionRequest collision_request;
	collision_request.group_name = "dual_arm";
	collision_detection::CollisionResult collision_result;

	robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
	current_state.setToDefaultValues();
		
	std::size_t count = 0;
	std::size_t milestone = 5000;
	
        std::string line;
	while(std::getline(joints_list, line))
	{
		if(count % milestone == 0){
		    ROS_INFO_STREAM("[" << count / 1000 << "%] Iteration " << std::to_string(count) << " out of 100.000");
		}		
		
		std::vector<double> joints;

                std::string joint;
                std::stringstream joints_stream(line);
		while(std::getline(joints_stream, joint, ',')){
		   joints.push_back( atof(joint.c_str()));
		}
		
		current_state.setVariablePositions(joints);		
		collision_result.clear();
		
		std::time_t t0;
		std::time_t t1;
		
		t0 = time(NULL);
		planning_scene.checkCollision(collision_request, collision_result);
		t1 = time(NULL) - t0;
		
		for (std::size_t i = 0; i < joints.size(); ++i)
		{
			csv_content << joints[i] << ";";
		}

		csv_content << not collision_result.collision << ";";
		csv_content << t1 << "\n";

		count = count + 1;
	}
	
	ROS_INFO_STREAM("[100%] Done!");

	// Write to the file
	output << csv_content.str();

	// Close the files
	joints_list.close();
	output.close();

	ros::shutdown();
	return 0;
}
