// David Dang djd122 12/7/18

// jenga_tower_builder: 
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

// launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// rosrun magic_object_finder magic_object_finder
// then run this node - rosrun cartesian_motion_commander jenga_tower_builder

// this node uses the carMoveActionServer along with magic_object_finder to identity target blocks and move them to assemble a small jenga tower

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<sensor_msgs/JointState.h>
//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>
using namespace std;

geometry_msgs::PoseStamped g_block_data;

//Hard-coded names of objects of interest
string g_object_name("toy_block");  //hard-coded object
string g_object_name2("toy_block2");
string g_object_name3("toy_block3");
string g_object_name4("toy_block4");
string g_object_name5("toy_block5");
string g_object_name6("toy_block6");
string g_object_name7("toy_block7");
string g_object_name8("toy_block8");
string g_object_name9("toy_block9");

int g_found_object_code; //global to communicate between callback and main: true if named object was found
geometry_msgs::PoseStamped g_perceived_object_pose; //global to communicate between callback and main: pose  of found object

ros::Publisher *g_pose_publisher; //make this global so callback can access it--for displaying object frames in rviz


//this callback function receives a result from the magic object finder action server
//it sets g_found_object_code to true or false, depending on whether the  object was found
//if the object was found, then components of g_perceived_object_pose are filled in
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    }
    else if (g_found_object_code==magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         g_pose_publisher->publish(g_perceived_object_pose);  //this is to enable display of pose of found object in rviz
    }
    else {
        ROS_WARN("object not found!");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jenga_tower_builder"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander; 
    XformUtils xformUtils; 
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/sticky_finger/link6");
    std_srvs::SetBool srv;
    srv.request.data = true;
    
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout=false; 
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder

	//if no warnings, then object was found
	//done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion 

    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;

    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    O_des << 0.5, 0.3, 0.3;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des; 
    
    //Hard coded toy_block destinations
    Eigen::Vector2d one_des, two_des, three_des, four_des, five_des, six_des, seven_des, eight_des, nine_des;
    one_des << 0.45, 0.4; // [x_des, y_des]
    two_des << 0.52, 0.4;
    three_des << 0.58, 0.4;
    four_des << 0.5, 0.46; 
    five_des << 0.5, 0.4;
    six_des << 0.5, 0.33;
    seven_des << 0.45, 0.4;
    eight_des << 0.52, 0.4;
    nine_des << 0.58, 0.4;

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }

    while (ros::ok()) {       
		//"go look for toy_block"
    	object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
        
        //move to approach pose:
        ROS_INFO("moving to approach pose");        
        //match x and y coordinates of target
        tool_pose.pose.position.x = g_perceived_object_pose.pose.position.x; 
        tool_pose.pose.position.y = g_perceived_object_pose.pose.position.y; 
        tool_pose.pose.position.z = 0.05;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }                   
        
        //lower tool to approach part to grasp
        ROS_INFO("lowering to grasp pose");         
        tool_pose.pose.position.z = 0.0340; //descend - block is 0.035 high      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
		
		//enable the vacuum gripper
        ROS_INFO("enabling vacuum gripper");       
        srv.request.data = true;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

		//depart after enabling vacuum gripper
        ROS_INFO("requesting plan to depart with grasped object:");
        tool_pose.pose.position.z = 0.3;         
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

		//move block to my specified location		
		ROS_INFO("Moving to target location");
		tool_pose.pose.position.x=one_des[0];
        tool_pose.pose.position.y=one_des[1]; 
        tool_pose.pose.orientation.x=0; // [0,0,0,1] menans perfectly aligned to base frame
        tool_pose.pose.orientation.y=0;
        tool_pose.pose.orientation.z=0;
        tool_pose.pose.orientation.w=1;    
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //disable the vacuum gripper
        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        
        //"go look for toy_block2"
    	object_finder_goal.object_name = g_object_name2.c_str(); //convert string object to old C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
        
        //move to approach pose:
        ROS_INFO("moving to approach pose");        
        //match x and y coordinates of target
        tool_pose.pose.position.x = g_perceived_object_pose.pose.position.x; 
        tool_pose.pose.position.y = g_perceived_object_pose.pose.position.y; 
        tool_pose.pose.position.z = 0.05;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }                   
        
        //lower tool to approach part to grasp
        ROS_INFO("lowering to grasp pose");         
        tool_pose.pose.position.z = 0.0340; //descend - block is 0.035 high      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
		
		//enable the vacuum gripper
        ROS_INFO("enabling vacuum gripper");       
        srv.request.data = true;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

		//depart after enabling vacuum gripper
        ROS_INFO("requesting plan to depart with grasped object:");
        tool_pose.pose.position.z = 0.3;         
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

		//move block to my specified location		
		ROS_INFO("Moving to target location");
		tool_pose.pose.position.x=two_des[0];
        tool_pose.pose.position.y=two_des[1];
        tool_pose.pose.orientation.x=0;  
        tool_pose.pose.orientation.y=0;
        tool_pose.pose.orientation.z=0;
        tool_pose.pose.orientation.w=1;     
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //disable the vacuum gripper
        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        
        //"go look for toy_block3"
    	object_finder_goal.object_name = g_object_name3.c_str(); //convert string object to old C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
        
        //move to approach pose:
        ROS_INFO("moving to approach pose");        
        //match x and y coordinates of target
        tool_pose.pose.position.x = g_perceived_object_pose.pose.position.x; 
        tool_pose.pose.position.y = g_perceived_object_pose.pose.position.y; 
        tool_pose.pose.position.z = 0.05;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }                   
        
        //lower tool to approach part to grasp
        ROS_INFO("lowering to grasp pose");         
        tool_pose.pose.position.z = 0.0340; //descend - block is 0.035 high      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
		
		//enable the vacuum gripper
        ROS_INFO("enabling vacuum gripper");       
        srv.request.data = true;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

		//depart after enabling vacuum gripper
        ROS_INFO("requesting plan to depart with grasped object:");
        tool_pose.pose.position.z = 0.3;         
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

		//move block to my specified location		
		ROS_INFO("Moving to target location");
		tool_pose.pose.position.x=three_des[0];
        tool_pose.pose.position.y=three_des[1];
        tool_pose.pose.orientation.x=0;  
        tool_pose.pose.orientation.y=0;
        tool_pose.pose.orientation.z=0;
        tool_pose.pose.orientation.w=1;     
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //disable the vacuum gripper
        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        //ROS_INFO("Jenga tower completed");
        //break; //break out of while loop once all blocks are set
    }

    return 0;
}

