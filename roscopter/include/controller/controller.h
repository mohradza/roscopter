#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/BtrajCommand.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/ControlStatus.h>
#include <z_state_estimator/ZStateEst.h>
#include <controller/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <stdint.h>
#include <dynamic_reconfigure/server.h>
#include <roscopter/ControllerConfig.h>

namespace controller
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
} state_t;

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double x_dot;
  double y_dot;
  double z_dot;

  double r;

  double ax;
  double ay;
  double az;

  double ax_ff; //mo added ff variables
  double ay_ff;
  double az_ff;
  double throttle;

} command_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double n_dot;
  double e_dot;
  double d_dot;
} max_t;

class Controller
{

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber btraj_cmd_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber attitude_sub_;
  ros::Subscriber z_est_sub_;
  ros::Subscriber control_status_sub_;
  ros::Publisher command_pub_;

  // Paramters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double max_accel_xy_;
  double max_accel_z_;
  double drag_constant_;
  double pn_dot_;
  double pe_dot_;
  
  double takeoff_limiter_;
  double takeoff_slew_rate_;
  double landing_limiter_;
  double landing_slew_rate_;
  double landed_limiter_;

  bool landed_;
  bool landing_;
  bool time_landing_;
  bool takeoff_;
  int control_status_;

  ros::WallTime start_time_;
  ros::WallTime current_time_;
  double flight_time_;
  double landing_time_;

  bool is_flying_;
  bool armed_;
  int ignore_;

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;
  controller::SimplePID PID_xtot_;//bnr - PID objects for single PD controller (vs cascade) for each dimension
  controller::SimplePID PID_ytot_;
  controller::SimplePID PID_ztot_;
  controller::SimplePID PID_xveltot_;
  controller::SimplePID PID_yveltot_;
  controller::SimplePID PID_zveltot_;

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<roscopter::ControllerConfig> _server;
  dynamic_reconfigure::Server<roscopter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(roscopter::ControllerConfig& config,
                            uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  max_t max_ = {};
  rosflight_msgs::Command command_;
  command_t xc_ = {}; // command
  double prev_time_;
  uint8_t control_mode_;
  uint8_t controller_select_;

  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void attitudeCallback(const geometry_msgs::Vector3StampedConstPtr &msg);
  void zStateCallback(const z_state_estimator::ZStateEstConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void btrajCmdCallback(const rosflight_msgs::BtrajCommandConstPtr &msg);
  void statusCallback(const rosflight_msgs::StatusConstPtr &msg);
  void controlStatusCallback(const rosflight_msgs::ControlStatusPtr &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
};
}

#endif
