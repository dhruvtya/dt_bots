#include "ros/ros.h"
#include "dtb_quadruped_states/QuadrupedBasicService.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <cmath>

using namespace std;

vector <vector<float>> motors_min_max
{
  {M_PI/10,-M_PI/10}, {-M_PI/10,M_PI/10}, {M_PI/10,-M_PI/10}, {-M_PI/10,M_PI/10},
  {0.0,-M_PI}, {0.0,M_PI}, {0.0,-M_PI}, {0.0,M_PI},
  {0.0,M_PI}, {0.0,-M_PI}, {0.0,M_PI}, {0.0,-M_PI},

};


// Global joint publisher variables
ros::Publisher t1_pub, t2_pub, t3_pub, t4_pub;
ros::Publisher l1_pub, l2_pub, l3_pub, l4_pub;
ros::Publisher h1_pub, h2_pub, h3_pub, h4_pub;



// Callback for basic service
bool handle_basic_service_request(dtb_quadruped_states::QuadrupedBasicService::Request& req,
    dtb_quadruped_states::QuadrupedBasicService::Response& res)
{
    std_msgs::Float64 t1_com, t2_com, t3_com, t4_com;
    std_msgs::Float64 l1_com, l2_com, l3_com, l4_com;
    std_msgs::Float64 h1_com, h2_com, h3_com, h4_com;
    
    switch(req.requested_state){
       case 0:
         ROS_INFO("Sit request");
         t1_com.data = 0;
         t2_com.data = 0;
         t3_com.data = 0;
         t4_com.data = 0;
         
         l1_com.data = motors_min_max[4].at(0);
         l2_com.data = motors_min_max[5].at(0);
         l3_com.data = motors_min_max[6].at(0);
         l4_com.data = motors_min_max[7].at(0);
         
         h1_com.data = motors_min_max[8].at(0);
         h2_com.data = motors_min_max[9].at(0);
         h3_com.data = motors_min_max[10].at(0);
         h4_com.data = motors_min_max[11].at(0);
    
       break;
       
       default:
         ROS_INFO("Invalid request, Sitting down");
         
    }
    
    t1_pub.publish(t1_com);
    t2_pub.publish(t2_com);
    t3_pub.publish(t3_com);
    t4_pub.publish(t4_com);
    
    l1_pub.publish(l1_com);
    l2_pub.publish(l2_com);
    l3_pub.publish(l3_com);
    l4_pub.publish(l4_com);
    
    h1_pub.publish(h1_com);
    h2_pub.publish(h2_com);
    h3_pub.publish(h3_com);
    h4_pub.publish(h4_com);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback =  "State executed"  ;
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv)
{
    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "dtbq_basic_servies");
    ros::NodeHandle n;
    

    // Publishers to publish std_msgs::Float64 messages on motor joints topics
    t1_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/t_motor_1_position_controller/command", 10);
    t2_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/t_motor_2_position_controller/command", 10);
    t3_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/t_motor_3_position_controller/command", 10);
    t4_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/t_motor_4_position_controller/command", 10);
    
    l1_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/l_motor_1_position_controller/command", 10);
    l2_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/l_motor_2_position_controller/command", 10);
    l3_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/l_motor_3_position_controller/command", 10);
    l4_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/l_motor_4_position_controller/command", 10);
    
    h1_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/h_motor_1_position_controller/command", 10);
    h2_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/h_motor_2_position_controller/command", 10);
    h3_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/h_motor_3_position_controller/command", 10);
    h4_pub = n.advertise<std_msgs::Float64>("/dtb_quadruped/h_motor_4_position_controller/command", 10);
    
    
    
    // Service with a handle_basic_service_request callback function
    ros::ServiceServer service = n.advertiseService("/dtb_quadruped/basic_service", handle_basic_service_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
