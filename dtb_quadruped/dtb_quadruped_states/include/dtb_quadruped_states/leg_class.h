#include "ros/ros.h"
#include <cmath>

using namespace std;

//// GLOBAL ///////////////////////
///////////////////////////////////


const double MAX_LEG_EXTENSION = 0.4;
const double MAX_PERP_HEIGHT = 0.35;
const double MAX_LR_ANGLE = M_PI/10;
const double MAX_FB_ANGLE = M_PI/9;
const double THIGH_LEN = 0.25;
const double CALF_LEN = 0.25;

vector <vector<float>> motors_min_max
{
  {M_PI/10,-M_PI/10}, {-M_PI/10,M_PI/10}, {M_PI/10,-M_PI/10}, {-M_PI/10,M_PI/10},
  {0.0,-M_PI}, {0.0,M_PI}, {0.0,-M_PI}, {0.0,M_PI},
  {0.0,M_PI}, {0.0,-M_PI}, {0.0,M_PI}, {0.0,-M_PI},
};


///////////////////////////////////
///////////////////////////////////

class Leg_Class
{
  private:
    int leg_no;

  public:
    Leg_Class(int input_leg_no)
    {
      leg_no = input_leg_no;
      ROS_INFO("Leg Class for leg no. %d initialised", leg_no);
    }
    
    
    
    double motor_angle_correction(char motor_type, double temp_angle, double temp_motor_angle)
    {
      switch(motor_type)
      {
        case 't':
          if(leg_no == 2 || leg_no == 4)
          {
            temp_motor_angle = -(temp_motor_angle);
          }
        break;
        
        case 'l':
          temp_motor_angle = temp_motor_angle + temp_angle;
          if(leg_no == 1 || leg_no == 3)
          {
            temp_motor_angle = -(temp_motor_angle);
          }
        break;
        
        case 'h':
          if(leg_no == 2 || leg_no == 4)
          {
            temp_motor_angle = -(temp_motor_angle);
          }
        break;
        
        default:
          ROS_ERROR("Invalid motor type for motor angle correction");
      }
      return temp_motor_angle;
    }
    
    
    
    vector <double> calc_motor_angle(double perp_height, double fb_angle, double lr_angle)
    {
      
      vector <double> motor_commands {0.0, 0.0, 0.0};
      
      if((perp_height < 0.0 || perp_height > MAX_PERP_HEIGHT) || (lr_angle > MAX_LR_ANGLE || lr_angle < (-MAX_LR_ANGLE)) || (fb_angle > MAX_FB_ANGLE || fb_angle < (-MAX_FB_ANGLE)))
      {
        ROS_ERROR("Leg Class Calculation exceeding limits - Perp_Height : %f, FB_Angle : %f, LR_Angle : %f", perp_height, fb_angle, lr_angle);
      }
      
      else
      {
        double theta_l = 0.0, theta_h = 0.0, theta_t = 0.0;
        double LEF = ((perp_height/cos(lr_angle)) + (0.1*(tan(lr_angle))))/cos(fb_angle);
        
        theta_l = asin((LEF/2)/THIGH_LEN);
        theta_h = 2 * theta_l;
        theta_t = lr_angle;
        
        theta_l = motor_angle_correction('l', fb_angle, theta_l);
        theta_h = motor_angle_correction('h', fb_angle, theta_h);
        theta_t = motor_angle_correction('t', lr_angle, theta_t);
        
        motor_commands[0] = theta_t;
        motor_commands[1] = theta_l;
        motor_commands[2] = theta_h;
      }
      
      return motor_commands;

    }

};
