#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>
#include <cstdlib> // For atof
ros::Publisher velocity_publisher;
ros::Subscriber current_turtle_pose;
turtlesim::Pose current_pose;
geometry_msgs::Twist velocity_msg;
bool subscribe_started = false;

using namespace std;

class GoalPose {
    public: 
        double x, y;

        GoalPose(int x, int y) {
            this->x = x;
            this->y = y;
        }  
};


void getCurrentPose(const turtlesim::Pose::ConstPtr& pose);
double degrees_to_radians(double degrees);
double radians_to_degrees(double degrees);
void stop();
double angleDifference(GoalPose& goal_pose);
void rotate(double angular_velocity, double rotate_angle_in_degrees, int direction);
void new_rotate(double angular_velocity, int direction, GoalPose& goal_pose);
void move(double linear_distance, double linear_velocity, int direction);
double calculate_euclidian_distance(GoalPose& goal_pose);
void move_to_goal(GoalPose& goal_pose);
void gas_pedal(double linear_velocity);
void steer(GoalPose& goal_pose, double angular_velocity, double current_angle_difference_in_degrees);


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_move");
    ros::NodeHandle n;

    cout << atoi(argv[1]) << endl;
    cout << atoi(argv[2]) << endl;

    
    GoalPose goal_pose(atoi(argv[1]), atoi(argv[2]));
    
    // Publisher for controlling turtle velocity
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Subscriber to get the current pose of the turtle
    current_turtle_pose = n.subscribe("/turtle1/pose", 10, getCurrentPose);

    // cout << goal_pose.x << endl;

    // move(3.0, 2, 1);
    // rotate(20.0, 90, 1);

    // new_rotate(10.0, 1, goal_pose);
    move_to_goal(goal_pose);

    return 0;
}


// Callback to update the current pose of the turtle
void getCurrentPose(const turtlesim::Pose::ConstPtr& pose) {
    current_pose = *pose;
    ROS_INFO("Current Pose -> x: %.2f, y: %.2f, theta: %.2f", current_pose.x, current_pose.y, current_pose.theta);
    subscribe_started = true;
}

double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radians_to_degrees(double degrees) {
    return degrees * 180.0 / M_PI;
}

void stop() {
    velocity_msg.linear.x = 0.0;
    velocity_msg.linear.y = 0.0;
    velocity_msg.linear.z = 0.0;
    velocity_msg.angular.x = 0.0;
    velocity_msg.angular.y = 0.0;
    velocity_msg.angular.z = 0.0;
    velocity_publisher.publish(velocity_msg);
}

double angleDifference(GoalPose& goal_pose) {
    return radians_to_degrees(atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x) - current_pose.theta); 
}

void rotate(double angular_velocity, double rotate_angle_in_degrees, int direction) {
    // move forward
    switch(direction) {
        case 1:
            velocity_msg.angular.z = abs(degrees_to_radians(angular_velocity));
            break;
        case 2:
            velocity_msg.angular.z = -abs(degrees_to_radians(angular_velocity)); 
            break;
    }

    velocity_msg.linear.x = 0;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;

    

    double initiated_time = ros::Time::now().toSec();
    double current_angle_in_radians = 0;
    double tolerance_distance = 0.1;

    ros::Rate loop_rate(10);
    do{
        velocity_publisher.publish(velocity_msg);
        double time_at_iteration = ros::Time::now().toSec();
        current_angle_in_radians = angular_velocity * (time_at_iteration - initiated_time);

        cout << "DISTANCE ROTATED :: " << current_angle_in_radians << endl;

        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle_in_radians < rotate_angle_in_degrees);
     // Set forward speed

    cout << "GOAL REACHED !!!" << endl;
    stop();
}

void new_rotate(double angular_velocity, int direction, GoalPose& goal_pose) {
    // move forward
    switch(direction) {
        case 1:
            velocity_msg.angular.z = abs(degrees_to_radians(angular_velocity));
            break;
        case 2:
            velocity_msg.angular.z = -abs(degrees_to_radians(angular_velocity)); 
            break;
    }

    velocity_msg.linear.x = 0;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;

    

    double tolerance_angle = 1.0;
    double current_angle_difference_in_radians = MAX_INPUT;

    ros::Rate loop_rate(10);
    do{
        velocity_publisher.publish(velocity_msg);
        // double time_at_iteration = ros::Time::now().toSec();
        // current_angle_in_radians = angular_velocity * (time_at_iteration - initiated_time);
        cout << "Rotation difference :: " << abs(angleDifference(goal_pose)) << endl;

        current_angle_difference_in_radians = angleDifference(goal_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }while(abs(current_angle_difference_in_radians) >= tolerance_angle);


    cout << "GOAL REACHED !!!" << endl;
    stop();
}

void move(double linear_distance, double linear_velocity, int direction) {
    // move forward
    switch(direction) {
        case 1:
            velocity_msg.linear.x = linear_velocity;
            break;
        case 2:
            velocity_msg.linear.x = -linear_velocity; 
            break;
    }

    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;
    velocity_msg.angular.z = 0;

    

    double initiated_time = ros::Time::now().toSec();
    double current_distance = 0;
    double tolerance_distance = 0.1;

    ros::Rate loop_rate(10);
    do{
        velocity_publisher.publish(velocity_msg);
        double time_at_iteration = ros::Time::now().toSec();
        current_distance = linear_velocity * (time_at_iteration - initiated_time);

        cout << "DISTANCE TRAVELLED" << current_distance << endl;

        ros::spinOnce();
        loop_rate.sleep();
    }while(current_distance < linear_distance);
     // Set forward speed

    cout << "GOAL REACHED !!!" << endl;
    stop();
}

double calculate_euclidian_distance(GoalPose& goal_pose) {
    return sqrt(pow((current_pose.x - goal_pose.x), 2)+ pow((current_pose.y - goal_pose.y), 2));
}

void move_to_goal(GoalPose& goal_pose) {

    ros::Rate loop_rate(10);
    do{
        if(subscribe_started) {

            double euclidian_distance = calculate_euclidian_distance(goal_pose);
            double tolerance_distance = 1.0;

            double current_angle_difference_in_degrees = angleDifference(goal_pose);
            double tolerance_angle = 1.0;

            cout << "euclidian_distance:: " << euclidian_distance << current_angle_difference_in_degrees << endl;

            if(euclidian_distance < tolerance_distance) break;

            double Kp_linear = 0.5;
            double Kp_angular = 0.623;

            gas_pedal(Kp_linear*euclidian_distance);
            steer(goal_pose, Kp_angular*degrees_to_radians(current_angle_difference_in_degrees), current_angle_difference_in_degrees);
            velocity_publisher.publish(velocity_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }while(ros::ok());

    stop();

}


void gas_pedal(double linear_velocity) {
    velocity_msg.linear.x = linear_velocity;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;
}


void steer(GoalPose& goal_pose, double angular_velocity, double current_angle_difference_in_degrees) {   
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;

    cout << "Current Angle Difference :: " << (current_angle_difference_in_degrees) << endl;

    if((current_angle_difference_in_degrees) > 0) {
        cout << "On LEFT" << endl;
        velocity_msg.angular.z = angular_velocity;
    } else if ((current_angle_difference_in_degrees) < 0) {
        cout << "On Right" << endl;
        velocity_msg.angular.z = -angular_velocity;
    } else {
       velocity_msg.angular.z = 0; 
    }
}
