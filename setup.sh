#!/bin/bash
: 'Explanation:
System and ROS Setup: The script updates the system and installs ROS Noetic.
Workspace and Package Creation: It sets up a workspace and a package tailored for C++ development with action functionality.
Action Definition and Compilation: Defines a custom action for a complex task and ensures all components are built.
Server and Client Code: Includes simple C++ templates for an action server and client that communicate over a custom action, showcasing the complete lifecycle of a ROS action.
Testing and Advanced Interaction: Instructs on how to test the setup and provides guidance on using ROS's command-line tools for deeper interaction and monitoring.
'
# Author: Prof. Mehdi Pirahandeh
# ROS script focusing on ROS Actions using C++.

echo "Setting up an advanced ROS action scenario for educational purposes..."

# 1. Update and install ROS Noetic, which is compatible with Ubuntu 20.04
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo "Installing ROS Noetic for C++ development..."
sudo apt install ros-noetic-desktop-full -y

# Setup your ROS environment
echo "Setting up ROS environment..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 2. Create a ROS workspace
echo "Creating a ROS workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# 3. Create a ROS package with necessary dependencies
echo "Creating a ROS package named 'advanced_actions' with C++ dependencies..."
cd src
catkin_create_pkg advanced_actions roscpp actionlib actionlib_msgs

cd advanced_actions

# 4. Add action file in the action directory
echo "Creating an action definition for a complex task..."
mkdir action
echo "string order
---
string outcome
---
string feedback" > action/ComplexTask.action

# 5. Modify package.xml to ensure all dependencies are correctly listed
echo "Updating package.xml to include necessary dependencies for action handling..."
sed -i '/<\/package>/i <build_depend>actionlib_msgs</build_depend>' package.xml
sed -i '/<\/package>/i <exec_depend>actionlib_msgs</exec_depend>' package.xml

# 6. Update CMakeLists.txt to handle the action files and link libraries
echo "Configuring CMakeLists.txt for C++ action files and node setup..."
sed -i '/find_package(catkin REQUIRED COMPONENTS/a find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib)' CMakeLists.txt
echo "add_action_files(DIRECTORY action FILES ComplexTask.action)" >> CMakeLists.txt
echo "generate_messages(DEPENDENCIES actionlib_msgs)" >> CMakeLists.txt

# 7. Building the package to include the new action file
echo "Building the ROS package..."
cd ~/catkin_ws
catkin_make

# 8. Source the setup file to ensure all paths are correctly set
source devel/setup.bash

# 9. Explanation and setup of C++ nodes
echo "Setting up C++ nodes for action server and client..."
echo "Server and Client programs handle the action logic, communicating via the ROS action protocol."

# C++ Server
echo "Creating a basic C++ action server..."
echo '#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <advanced_actions/ComplexTaskAction.h>

typedef actionlib::SimpleActionServer<advanced_actions::ComplexTaskAction> Server;

void execute(const advanced_actions::ComplexTaskGoalConstPtr& goal, Server* as) {
  // Simulate some work
  ros::Duration(2).sleep();

  // Feedback
  advanced_actions::ComplexTaskFeedback feedback;
  feedback.feedback = "Processing " + goal->order;
  as->publishFeedback(feedback);

  // Result
  advanced_actions::ComplexTaskResult result;
  result.outcome = "Completed " + goal->order;
  as->setSucceeded(result);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "complex_task_server");
  ros::NodeHandle n;
  Server server(n, "do_complex_task", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}' > src/complex_task_server.cpp

# C++ Client
echo "Creating a basic C++ action client..."
echo '#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <advanced_actions/ComplexTaskAction.h>
typedef actionlib::SimpleActionClient<advanced_actions::ComplexTaskAction> Client;

int main(int argc, char** argv) {
  ros::init(argc, argv, "complex_task_client");
  Client client("do_complex_task", true);
  client.waitForServer();

  advanced_actions::ComplexTaskGoal goal;
  goal.order = "Execute Order 66";
  client.sendGoal(goal);

  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The task was completed successfully.");
  else
    ROS_INFO("The task failed to complete.");

  return 0;
}' > src/complex_task_client.cpp

# 10. Build the nodes
echo "Building the action server and client nodes..."
cd ~/catkin_ws
catkin_make

# 11. Explain test scenario
echo "Test the action server and client by running the following commands in separate terminals:"
echo "rosrun advanced_actions complex_task_server"
echo "rosrun advanced_actions complex_task_client"

# 12. Advanced CLI Interaction
echo "You can interact with the ROS system using CLI tools like rostopic, rosnode, and rosservice for detailed system introspection."

# The script ends here but assumes the user has permissions and tools installed.


