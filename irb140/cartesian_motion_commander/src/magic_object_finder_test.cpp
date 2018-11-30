//Start the magic object finder action server: rosrun magic_object_finder magic_object_finder
//then run this node

#include <ros/ros.h>
using namespace std;
#include <geometry_msgs/PoseStamped.h>
//add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>

//another magic value: hard-coded name of object of interest
string g_object_name("toy_block");  //hard-coded object name; edit this for different objects
string g_object_name2("toy_block2");
string g_object_name3("toy_block3");
// name is "gear_part" on my virtual machine
int g_found_object_code; //global to communicate between callback and main: true if named object was found
geometry_msgs::PoseStamped g_perceived_object_pose; //global to communicate between callback and main: pose  of found object

ros::Publisher *g_pose_publisher;


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
    ros::init(argc, argv, "magic_object_finder_test"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
	//set up an action client to query object poses using the magic object finder
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction> object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout=false; 
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5));  
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal object_finder_goal; //instantiate goal message to communicate with magic_object_finder

		//the following makes an inquiry for the pose of the part of interest
    	//specify the part name, send it in the goal message, wait for and interpret the result
    

		//ROS_WARN("Looking for toy_block1");
		//"go look for toy_block"
    	object_finder_goal.object_name = g_object_name.c_str(); //convert string object to old     C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object 	finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
    //check the result code to see if object was found or not
    	/*if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        	ROS_INFO("found object!");
    	}    
    	else {
        	ROS_WARN("object not found!  Quitting");
        	return 1;
    	}*/

		//ROS_WARN("Looking for toy_block2");
		//"go look for toy_block2"
    	object_finder_goal.object_name = g_object_name2.c_str(); //convert string object to old     C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object 	finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
    //check the result code to see if object was found or not
    	/*if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        	ROS_INFO("found object!");
    	}    
    	else {
        	ROS_WARN("object not found!  Quitting");
        	return 1;
    	}*/

		//ROS_WARN("Looking for toy_block3");
		//"go look for toy_block3"
    	object_finder_goal.object_name = g_object_name3.c_str(); //convert string object to old     C-style string data
    	object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); //request object 	finding via action server
		finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0)); //wait for a max time for response
    //NOTE: could do something else here (if useful) while waiting for response from action server
    	if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result "); //this should not happen; should get result of found or not-found
            return 1;
        }
    //check the result code to see if object was found or not
    /*	if (g_found_object_code == magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        	ROS_INFO("found object!");
    	}    
    	else {
        	ROS_WARN("object not found!  Quitting");
        	return 1;
    	}*/

//if no warnings, then object was found
//done with inquiry.  If here, then part pose is in g_perceived_object_pose.  Use it to compute robot motion 

	return 0;
}
