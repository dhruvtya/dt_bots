#include "ros/ros.h"
#include "dtb_quadruped_states/QuadrupedBasicService.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
using namespace std;



//// GLOBAL ///////////////////////
///////////////////////////////////

// Global joint publisher variable
ros::Publisher motor_pub;

///////////////////////////////////
///////////////////////////////////



// Supporting functions
double leg_angle_calc(char, int, double);


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
         
         for(int i = 1; i <= 4; i++){
           motors_command[i+3] = leg_angle_calc('l', i, req.req_height);
           motors_command[i+7] = leg_angle_calc('h', i, req.req_height);
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



double leg_angle_calc(char motor_type, int leg_no, double height)
{
    double mot_angle = 0.0, theta_l;
    
    if((motor_type != 'l' && motor_type != 'h') || (leg_no > 4 || leg_no < 1) || (height < 0.05 || height > max_height)){
      ROS_INFO("Invalid command");
    }
    else{
      theta_l = asin((height/2)/thigh_len);
      if(motor_type == 'l'){
        if(leg_no == 1 || leg_no == 3){
          mot_angle = (-1)*theta_l;
        }
        else{mot_angle = theta_l;}
      }
      else{
        if(leg_no == 1 || leg_no == 3){
          mot_angle = 2*theta_l;
        }
        else{mot_angle = (-1)*(2*theta_l);}
      }
    }
    return mot_angle; 
}



