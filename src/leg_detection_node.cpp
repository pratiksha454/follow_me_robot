#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <leg_detector/leg_detector.h> // Include the leg_detector package

class LegDetectionNode {
public:
    LegDetectionNode() {
        leg_sub = nh.subscribe("/leg_tracker/tracks", 1, &LegDetectionNode::legCallback, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        target_distance = 1.0; // Adjust this value based on your preference
    }

    void legCallback(const leg_detector::LegArray::ConstPtr& leg_array) {
        if (!leg_array->legs.empty()) {
            // Assuming leg_array contains leg positions in the leg_detector frame
            const leg_detector::Leg& closest_leg = leg_array->legs.front();
            
            // Calculate the angle to the closest leg
            float leg_angle = atan2(closest_leg.y, closest_leg.x);
            
            // Calculate desired robot movement based on leg angle and target_distance
            float angular_velocity = leg_angle * 0.1; // Adjust the scaling factor
            
            // Publish the Twist message to control the robot's movement
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = 0.2; // Adjust the linear velocity
            cmd_msg.angular.z = angular_velocity;
            cmd_pub.publish(cmd_msg);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber leg_sub;
    ros::Publisher cmd_pub;
    float target_distance;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "leg_detection_node");
    LegDetectionNode leg_detection_node;
    ros::spin();
    return 0;
}
