#include "ros/ros.h"
#include "dtb_quadruped_states/QuadrupedBasicService.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include "dtb_quadruped_states/leg_class.h"
using namespace std;



//// GLOBAL ///////////////////////
///////////////////////////////////

// Global joint publisher variable
ros::Publisher motor_pub;

Leg_Class leg1(1), leg2(2), leg3(3), leg4(4);
Leg_Class legs[] {leg1, leg2, leg3, leg4};

///////////////////////////////////
///////////////////////////////////


// Callback for basic service
bool handle_basic_service_request(dtb_quadruped_states::QuadrupedBasicService::Request& req,
    dtb_quadruped_states::QuadrupedBasicService::Response& res)
{
    vector <double> motors_command {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    std_msgs::Float64MultiArray to_motor;
    
    switch(req.requested_state){
       case 0:
         ROS_INFO("Sit request");
         
         for(int i = 4; i<12; i++) {motors_command[i] = motors_min_max[i].at(0);}
    
       break;
       
       case 1:
         ROS_INFO("Stand request");
         for(int i = 0; i < 4; i++)
         {
           vector <double> temp_for_motor = legs[i].calc_motor_angle(req.req_height, 0.0, 0.0);
           motors_command[i] = temp_for_motor[0];
           motors_command[i+4] = temp_for_motor[1];
           motors_command[i+8] = temp_for_motor[2];
         }
       break;
       
       default:
         ROS_INFO("Invalid request");
         
    }
    
    to_motor.data = motors_command;
    
    // Publish to motor
    motor_pub.publish(to_motor);

    // Wait seconds for motors to settle
    ros::Duration(2).sleep();

    // Return a response message
    res.msg_feedback =  "State executed"  ;
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize the node and create a handle to it
    ros::init(argc, argv, "dtbq_basic_services");
    ros::NodeHandle n;
    

    // Publisher for motor joint group
    motor_pub = n.advertise<std_msgs::Float64MultiArray>("/dtb_quadruped/motors_group_position_controller/command", 10);
    
    // Service with a handle_basic_service_request callback function
    ros::ServiceServer service = n.advertiseService("/dtb_quadruped/basic_service", handle_basic_service_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}


