#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

class ObstacleDetectionNode {
public:
    ObstacleDetectionNode() {
        scan_sub = nh.subscribe("/scan", 1, &ObstacleDetectionNode::scanCallback, this);
        obstacle_pub = nh.advertise<std_msgs::Bool>("/obstacle_detected", 1);
        
        obstacle_distance = 0.5; // Adjust this value based on your preference
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_data) {
        // Assuming scan_data contains laser scan data
        // Note: This is a simple obstacle detection based on distance thresholds
        
        // Determine if an obstacle is detected
        bool obstacle_detected = false;
        for (const float distance : scan_data->ranges) {
            if (distance < obstacle_distance) {
                obstacle_detected = true;
                break;
            }
        }
        
        // Publish the obstacle detection status
        std_msgs::Bool obstacle_msg;
        obstacle_msg.data = obstacle_detected;
        obstacle_pub.publish(obstacle_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher obstacle_pub;
    float obstacle_distance;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection_node");
    ObstacleDetectionNode obstacle_detection_node;
    ros::spin();
    return 0;
}
