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

  // put the pickup object on Rviz
  int m_id = 0;
  goal.showMarker(my_goals[0].x, my_goals[0].y, my_goals[0].w, m_id);
  ros::Duration(5).sleep();
  goal.deleteMarker(m_id);
  ros::Duration(5).sleep();
  goal.showMarker(my_goals.back().x, my_goals.back().y, my_goals.back().w, m_id + 1);
  
  return 0;
}