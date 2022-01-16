#include "pick_objects/object_picker_library.h"

struct goalStruct {
  float x;
  float y;
  float w;
};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node

  vector<goalStruct> my_goals;
  goalStruct g1{0.5, -4.15, 1.0};
  goalStruct g2{-5.31, 2.99, 1.0};

  my_goals.push_back(g1);
  my_goals.push_back(g2);

  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  ObjectPicker goal(&n);

  bool missionComplete = true;

  // put the pickup object on Rviz
  int m_id = 0;
  goal.showMarker(my_goals[0].x, my_goals[0].y, my_goals[0].w, m_id);

  for (int i = 0; i < my_goals.size(); ++i) {
    // set a new goal
    goal.sendGoal(my_goals[i].x, my_goals[i].y, my_goals[i].w);
    

    // wait for the goal status
    if (!goal.goalAchieved()) {
      ROS_INFO_STREAM(ros::this_node::getName() << "node: the robot failed to complete the mession.");
      missionComplete = false;
      break;
    }
    
    // pick up the object
    if (i == 0) {
      goal.deleteMarker(m_id);
    }

  }


  
  if (missionComplete) {
    ROS_INFO_STREAM(ros::this_node::getName() << "node: the mission is complete!");

    // pick down the object
    goal.showMarker(my_goals.back().x, my_goals.back().y, my_goals.back().w, m_id + 1);
  } else {
    ROS_INFO_STREAM(ros::this_node::getName() << "node: the mission is failed!");
  }


  return 0;
}