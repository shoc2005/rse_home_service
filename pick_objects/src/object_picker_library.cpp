#include "pick_objects/object_picker_library.h"




// define the class constructor
ObjectPicker::ObjectPicker(ros::NodeHandle* nh) {
    ac_ = new MoveBaseClient("move_base", true);

    while(!ac_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    client_set_marker_ = nh->serviceClient<add_markers::SetMarker>("/add_markers/set_marker");
    client_del_marker_ = nh->serviceClient<add_markers::DeleteMarker>("/add_markers/del_marker");

}

ObjectPicker::ObjectPicker(ros::NodeHandle* nh, string frame_id) {
    ac_ = new MoveBaseClient("move_base", true);

    while(!ac_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    frame_id_ = frame_id;

    client_set_marker_ = nh->serviceClient<add_markers::SetMarker>("/add_markers/set_marker");
    client_del_marker_ = nh->serviceClient<add_markers::DeleteMarker>("/add_markers/del_marker");
}

ObjectPicker::~ObjectPicker() {
    delete ac_;
}

// This function should publish the requested goal.
// After publishing the goal and reaching it, a message feedback should be returned with the requested goal
void ObjectPicker::sendGoal(float x, float y, float orientation)
{
    // set up the frame parameters
    goal_.target_pose.header.frame_id = frame_id_;
    goal_.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;
    goal_.target_pose.pose.orientation.w = orientation;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO_STREAM(ros::this_node::getName() << "node: sending a new goal: x=" << x << ", y="<< y << ", orientation=" << orientation);
    ac_->sendGoal(goal_);
}

// Check if the last goal is achieved
bool ObjectPicker::goalAchieved(void) {
    
    if (ac_->getState() != actionlib::SimpleClientGoalState::ACTIVE && ac_->getState() != actionlib::SimpleClientGoalState::PENDING){
    return(false);
    }

    // wait until trying to achieve the goal
    ac_->waitForResult();

    // check state of the goal
    if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    
    return(true);
    }
    else {
    
    return(false);
    }
}

// Show the marker in Rviz
void ObjectPicker::showMarker(float x, float y, float orientation, int marker_id) {

    add_markers::SetMarker srv;    

    srv.request.x = x;
    srv.request.y = y;
    srv.request.w = orientation;
    srv.request.id = marker_id;

    if (!client_set_marker_.call(srv)) {
        ROS_INFO("Error: set a new marker in Rviz");
    }
}

// Delete the marker in Rviz
void ObjectPicker::deleteMarker(int marker_id) {

    add_markers::DeleteMarker srv;    

    
    srv.request.id = marker_id;

    if (!client_del_marker_.call(srv)) {
        ROS_INFO("Error: delete the marker in Rviz");
    }
}