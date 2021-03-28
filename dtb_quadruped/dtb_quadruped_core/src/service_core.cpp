#include "ros/ros.h"
#include "dtb_quadruped_core/CoreService.h"
#include "dtb_quadruped_states/QuadrupedBasicService.h"

// Clients
ros::ServiceClient basic_service_client;

// Service core callback
bool handle_service_core_request(dtb_quadruped_core::CoreService::Request& req, dtb_quadruped_core::CoreService::Response& res)
{
    if(req.requested == "sit")
    {
      dtb_quadruped_states::QuadrupedBasicService sit_request;
      sit_request.request.requested_state = 0;
      sit_request.request.req_height = 0.0;
      ROS_INFO("Requesting - SIT");
      basic_service_client.call(sit_request);
    }
    else if(req.requested == "stand")
    {
      dtb_quadruped_states::QuadrupedBasicService stand_request;
      stand_request.request.requested_state = 1;
      stand_request.request.req_height = req.param;
      ROS_INFO("Requesting - STAND Height : %f", req.param);
      basic_service_client.call(stand_request);
    }
    return true;
}

int main(int argc, char** argv)
{
    // Initialize the node and create a handle to it
    ros::init(argc, argv, "dtbq_service_core");
    ros::NodeHandle n;
    
    // Service core
    ros::ServiceServer service = n.advertiseService("/dtb_quadruped/service_core", handle_service_core_request);
    ROS_INFO("Ready to control DTB Quadruped");
    
    // Clients for services
    basic_service_client = n.serviceClient<dtb_quadruped_states::QuadrupedBasicService>("/dtb_quadruped/basic_service");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
