#ifndef OBJECT_PICKER_ROSCPP_LIBRARY_H
#define OBJECT_PICKER_ROSCPP_LIBRARY_H

#include <ros/ros.h>
#include "add_markers/SetMarker.h"
#include "add_markers/DeleteMarker.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> 
#include <string.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

class ObjectPicker
{
private:
    move_base_msgs::MoveBaseGoal goal_;
    string frame_id_ = "map";
    MoveBaseClient *ac_;
    ros::ServiceClient client_set_marker_;
    ros::ServiceClient client_del_marker_;

public:
    ObjectPicker(ros::NodeHandle* nh);
    ObjectPicker(ros::NodeHandle* nh, string frame_id);
     ~ObjectPicker();
    void sendGoal(float x, float y, float orientation);
    bool goalAchieved(void);
    void showMarker(float x, float y, float orientation, int marker_id);
    void deleteMarker(int marker_id);
};

#endif