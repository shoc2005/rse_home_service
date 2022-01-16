#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
#include "add_markers/SetMarker.h"
#include "add_markers/DeleteMarker.h"

using namespace std;

class AddMarkers {
    public:
        // The class constructor
        AddMarkers(ros::NodeHandle* nh, string frame_id) {

            // Setup /add_markers/set_marker service with the handle_set_marker callback function
            service_a_ = nh->advertiseService("/add_markers/set_marker", &AddMarkers::handle_set_marker, this);

            // Setup /add_markers/del_marker service with the handle_remove_marker callback function
            service_b_ = nh->advertiseService("/add_markers/del_marker", &AddMarkers::handle_remove_marker, this);

            // Inform the core about publishing marker messages
            marker_pub_ = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
            frame_id_ = frame_id;

        }

        // This function should publish the requested marker position in Rviz
        bool handle_set_marker(add_markers::SetMarker::Request& req, add_markers::SetMarker::Response& res)
        {

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.id = req.id;
            marker.type = shape_;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = req.x;
            marker.pose.position.y = req.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = req.w;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            marker_pub_.publish(marker);
            res.msg_feedback = "The marker ID=" + std::to_string(req.id) + " was set:   x=" + std::to_string(req.x) + ", y=" + std::to_string(req.y) + ", orientation=" + std::to_string(req.w);
            ROS_INFO_STREAM(res.msg_feedback);
            markers_.push_back(marker);
            
            return true;
        }

        // This function removes the marker from rViz
        bool handle_remove_marker(add_markers::DeleteMarker::Request& req, add_markers::DeleteMarker::Response& res) 
        {
            for (int i=0; i < markers_.size(); ++i) {
                if(markers_[i].id == req.id) {
                    markers_[i].action = visualization_msgs::Marker::DELETE;

                    marker_pub_.publish(markers_[i]);

                    res.msg_feedback = "The marker ID=" + std::to_string(req.id) + " was deleted";
                    ROS_INFO_STREAM(res.msg_feedback);

                    // delete the marker from the vector
                    markers_.erase(markers_.begin() + i);

                    return true;
                }
            }

            res.msg_feedback = "The marker ID=" + std::to_string(req.id) + "not found.";
            ROS_INFO_STREAM(res.msg_feedback);
            return false;

        }

    private:
        
        uint32_t shape_ = visualization_msgs::Marker::SPHERE;
        ros::ServiceServer service_a_, service_b_;
        ros::Publisher marker_pub_;
        string frame_id_;
        vector<visualization_msgs::Marker> markers_;   
        
};


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "add_markers");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    AddMarkers m_srv(&n, "map");
    
    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}