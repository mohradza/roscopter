#include <controller/controller.h>
#include <stdio.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // Retrieve global MAV equilibrium throttle. This is the only MAV specific
  // parameter that is required
  ros::NodeHandle nh_mav(ros::this_node::getNamespace());
  if (!nh_private_.getParam("equilibrium_throttle", throttle_eq_))
    ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  max_accel_z_ = 1.0 / throttle_eq_;
  max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_ / sqrt(2.);

  // Landing / takeoff variables
  is_flying_ = false;

  timed_landing_ = false;

  STATE_LANDED_ = false;
  STATE_TAKEOFF_ = false;
  STATE_HOVER_ = false;
  STATE_TRAJ_ = false;
  STATE_TRAJ_TO_STATE_HOVER_ = false;
  STATE_LANDING_ = false;

  control_status_ = 0;

  limiter_ = 0.0;
  takeoff_slew_rate_ = 0.010;
  landing_slew_rate_ = -0.00333;

  start_time_ = ros::WallTime::now();
  landing_time_ = 20;

  nh_private_.getParam("max_roll", max_.roll);
  nh_private_.getParam("max_pitch", max_.pitch);
  nh_private_.getParam("max_yaw_rate", max_.yaw_rate);
  nh_private_.getParam("max_throttle", max_.throttle);
  nh_private_.getParam("max_n_dot", max_.n_dot);
  nh_private_.getParam("max_e_dot", max_.e_dot);
  nh_private_.getParam("max_d_dot", max_.d_dot);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("vins_estimator/odometry", 1, &Controller::stateCallback, this);
  z_est_sub_ = nh_.subscribe("z_state_estimator/z_state_estimate", 1, &Controller::zStateCallback, this);
  attitude_sub_ = nh_.subscribe("/attitude/euler", 1, &Controller::attitudeCallback, this);
  is_flying_sub_ =
      nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  btraj_cmd_sub_ =
      nh_.subscribe("btraj_command", 1, &Controller::btrajCmdCallback, this);
  cmd_sub_ =
      nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &Controller::statusCallback, this);
  control_status_sub_ = nh_.subscribe("control_status", 1, &Controller::controlStatusCallback, this);
  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
}

void Controller::attitudeCallback(const geometry_msgs::Vector3StampedConstPtr &msg)
{
    // We want to use ROSCopters attiude estimates vs VINS-mono
    xhat_.phi = msg->vector.x;
    xhat_.theta = msg->vector.y;
}

void Controller::zStateCallback(const z_state_estimator::ZStateEstConstPtr &msg)
{
    xhat_.pd = -1*msg->height_agl.data;
}

void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This is coming directly from VINS-mono
  // VINS-mono seems to be outputting in NWU
  // All following transformations account for this
  // difference

  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = -msg->pose.pose.position.y;
  //xhat_.pd = -msg->pose.pose.position.z;

  // Our trajectory controller needs velocity
  // in the world frame.
  pn_dot_ = msg->twist.twist.linear.x;
  pe_dot_ = -msg->twist.twist.linear.y;

  // Convert Quaternion to RPY
  double roll;
  double pitch;
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(roll, pitch, xhat_.psi);

  // For some reason, the roll angle estimate from VINS-mono
  // is centered around +/- pi. This should correct this value
 //if (xhat_.phi > 0){
  //   xhat_.phi = xhat_.phi - M_PI;
  //} else if (xhat_.phi < 0){
  //   xhat_.phi = xhat_.phi + M_PI;
  //}

  //ROS_INFO("%f", xhat_.psi);

  // Negative signs are to correct for frame mismatch
  // between vicon and NED
  //xhat_.theta = -xhat_.theta;
  xhat_.psi = -xhat_.psi;
  float yaw = xhat_.psi;

  xhat_.u = cos(yaw)*msg->twist.twist.linear.x-sin(yaw)*msg->twist.twist.linear.y;
  xhat_.v = -1*(sin(yaw)*msg->twist.twist.linear.x+cos(yaw)*msg->twist.twist.linear.y);
  xhat_.w = -msg->twist.twist.linear.z;

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = -msg->twist.twist.angular.y;
  xhat_.r = -msg->twist.twist.angular.z;

  //if(is_flying_ && armed_)
  if(armed_)
  {
    ROS_WARN_ONCE("CONTROLLER ACTIVE");
    computeControl(dt);
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}

void Controller::controlStatusCallback(const rosflight_msgs::ControlStatusPtr &msg)
{
    control_status_ = msg->control_status;
    //0: On Ground, 1: Takeoff, 2: Hover, 3: Turn, 4: Trajectory, 5: Landing
    if(control_status_ == 0) // LANDED
    {
      STATE_LANDED_ = true;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = false;
    }
    else if(control_status_ == 1) // TAKEOFF
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = true;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = false;
    }
    else if(control_status_ == 2) // HOVER
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = true;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = false;
    }
    else if(control_status_ == 3) // TRAJECTORY TO HOVER
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = false;
    }
    else if(control_status_ == 4) // TRAJECTORY TO HOVER
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = true;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = false;
    }
    else if(control_status_ == 5) // LANDING
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = true;
      STATE_LANDING_ = true;
    }
    else if(control_status_ == 6) // ERROR (LAND VEHICLE)
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = true;
      ROS_WARN_ONCE("CONTROL STATUS ERROR = 6 (STATE MACHINE ERROR) --> LAND VEHICLE");
    }
    else // ERROR
    {
      STATE_LANDED_ = false;
      STATE_TAKEOFF_ = false;
      STATE_HOVER_ = false;
      STATE_TRAJ_ = false;
      STATE_TRAJ_TO_STATE_HOVER_ = false;
      STATE_LANDING_ = true;
      ROS_WARN_ONCE("CONTROL STATUS ERROR (VALUE OUTSIDE EXPECTED RANGE) --> LAND VEHICLE");
    }

}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
}

void Controller::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
    armed_ = msg->armed;
}

void Controller::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      xc_.pn = msg->x;
      xc_.pe = msg->y;
      xc_.pd = msg->F;
      xc_.psi = msg->z;
      ignore_ = msg->ignore;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      xc_.x_dot = msg->x;
      xc_.y_dot = msg->y;
      xc_.pd = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
      xc_.ax = msg->x;
      xc_.ay = msg->y;
      xc_.az = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
                msg->mode);
      break;
  }
}

void Controller::btrajCmdCallback(const rosflight_msgs::BtrajCommandConstPtr &msg)
{
      // Minus signs are to convert from
      // vicon / octomap NWU frame to NED
      xc_.pn = msg->x;
      xc_.pe = -msg->y;
      xc_.pd = -msg->F;
      xc_.psi = -msg->z;

      // These are actually pn_dot and pe_dot
      xc_.x_dot = msg->x_vel;
      xc_.y_dot = -msg->y_vel;
      xc_.z_dot = -msg->z_vel;

      xc_.ax_ff = msg->x_acc;
      xc_.ay_ff = msg->y_acc;
      xc_.az_ff = msg->z_acc;

      ignore_ = msg->ignore;
      control_mode_ = msg->mode;
      controller_select_ = msg->controller_select; //mo variable for switching controllers
}
void Controller::reconfigure_callback(roscopter::ControllerConfig& config,
                                      uint32_t level)
{
  double P, I, D, tau;
  double Kpx, Kdx, Kpy, Kdy, Kpz, Kdz; //bnr gains for a single PD loop in each dimension
  double Kpdx, Kpdy, Kpdz;
  double Kdvx, Kdvy, Kdvz;

	 double gain_scaler = .25;

  Kpx = config.xptot;
  Kpy = config.yptot;
  Kpz = config.zptot;

  Kpdx = config.kpdx;
  Kpdy = config.kpdy;
  Kpdz = config.kpdz;

  Kdvx = config.kdvx;
  Kdvy = config.kdvy;
  Kdvz = config.kdvz;

  Kdx = config.xdtot;
  Kdy = config.ydtot;
  Kdz = config.zdtot;

  // ROS_INFO("Kpdx: %f", Kpdx);

  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  max_.n_dot = config.max_n_dot;
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  max_.e_dot = config.max_e_dot;
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);

  P = config.down_P;
  I = config.down_I;
  D = config.down_D;
  max_.d_dot = config.max_d_dot;
  PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  PID_xtot_.setGains(Kpx, Kpdx, 0.025, tau, max_.n_dot, -max_.n_dot); //bnr - set values for PD controller
  PID_ytot_.setGains(Kpy, Kpdy, 0.025, tau, max_.e_dot, -max_.e_dot);
  PID_ztot_.setGains(Kpz, Kpdz, 0.05, tau);

  PID_xveltot_.setGains(Kdx, Kdvx, 0, tau, max_.n_dot, -max_.n_dot);//bnr - set values for PD controller
  PID_yveltot_.setGains(Kdy, Kdvy, 0, tau, max_.e_dot, -max_.e_dot);
  PID_zveltot_.setGains(Kdz, Kdvz, 0, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;

  max_.n_dot = config.max_n_dot;
  max_.e_dot = config.max_e_dot;
  max_.d_dot = config.max_d_dot;

  throttle_eq_ = config.equilibrium_throttle;

  ROS_INFO("new gains");
  resetIntegrators();
}

void Controller::computeControl(double dt)
{
    if(dt <= 0.0000001)
    {
      // This messes up the derivative calculation in the PID controllers
      return;
    }

    uint8_t mode_flag = control_mode_;
    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
        xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
        xc_.psi -= 2*M_PI;
    }
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

    // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
    // Rotate body frame velocities to vehicle 1 frame velocities
    double sinp = sin(xhat_.phi);
    double cosp = cos(xhat_.phi);
    double sint = sin(xhat_.theta);
    double cost = cos(xhat_.theta);
    double pxdot = cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
    double pydot = cosp * xhat_.v - sinp * xhat_.w;
    double pddot = -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

    // Nested Loop for Altitude
    double pddot_c = xc_.z_dot;
    double aff_scale = 0.0;

    //NEED TO DO:
    //Assign reference velocities from trajectories to xc_.x_dot, xc_.y_dot, pddot_c
    //Assign feedforward accelerations to xff, yff, zff;
    //double xvel_tot = PID_xveltot_.computePID(xc_.x_dot, pxdot, dt);
    //double yvel_tot = PID_yveltot_.computePID(xc_.y_dot, pydot, dt);

    double ax = 0.0;
    double ay = 0.0;

    ax = PID_xtot_.computePID(xc_.pn, xhat_.pn, dt) + PID_xveltot_.computePID(xc_.x_dot, pn_dot_, dt) + aff_scale*xc_.ax_ff;
    ay = PID_ytot_.computePID(xc_.pe, xhat_.pe, dt) + PID_yveltot_.computePID(xc_.y_dot, pe_dot_, dt) + aff_scale*xc_.ay_ff;
    xc_.az = PID_ztot_.computePID(xc_.pd, xhat_.pd, dt)+PID_zveltot_.computePID(pddot_c, pddot, dt) + aff_scale*xc_.az_ff;

    // Transform the desired accelerations to the body frame
    xc_.ax	= ax*cos(xhat_.psi) + ay*sin(xhat_.psi);
    xc_.ay	= -ax*sin(xhat_.psi) + ay*cos(xhat_.psi);

    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    double total_acc_c = sqrt((1.0 - xc_.az) * (1.0 - xc_.az) +
                              xc_.ax * xc_.ax + xc_.ay * xc_.ay);  // (in g's)
    if (total_acc_c > 0.001)
    {
        xc_.phi = asin(xc_.ay / total_acc_c);
        xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    } else {
        xc_.phi = 0;
        xc_.theta = 0;
    }

    // Compute desired thrust based on current pose
    xc_.throttle = (1.0 - xc_.az) * throttle_eq_ / cosp / cost;

    // Pack up and send the command
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.ignore = ignore_;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);

    // Handle takeoffs and landings
    //if((flight_time_ >= landing_time_) && (flight_time_ < landing_time_ + 1.0))
    //{
    //	      timed_landing_= 1;
    //}

    if(STATE_LANDED_)
    {
      limiter_ = 0;
      ROS_INFO_THROTTLE(1,"command_.F before limiting (STATE_LANDED_): %f", command_.F);
      command_.F = limiter_ * command_.F;
      ROS_INFO_THROTTLE(1,"command_.F after  limiting (STATE_LANDED_): %f", command_.F);
      ROS_INFO_THROTTLE(1,"limiter (takeoff): %f", limiter_);
    }
    if(STATE_TAKEOFF_)
    {
    	limiter_ = limiter_ + takeoff_slew_rate_;
      ROS_INFO_THROTTLE(1,"command_.F before limiting (STATE_TAKEOFF_): %f", command_.F);
      command_.F = limiter_ * command_.F;
      ROS_INFO_THROTTLE(1,"command_.F after  limiting (STATE_TAKEOFF_): %f", command_.F);
      ROS_INFO_THROTTLE(1,"limiter (takeoff): %f", limiter_);
    }
    if(STATE_HOVER_ && limiter_ < 1)
    {
    	limiter_ = limiter_ + takeoff_slew_rate_;
      ROS_INFO_THROTTLE(1,"command_.F before limiting (STATE_HOVER_): %f", command_.F);
      command_.F = limiter_ * command_.F;
      ROS_INFO_THROTTLE(1,"command_.F after  limiting (STATE_HOVER_): %f", command_.F);
      ROS_INFO_THROTTLE(1,"limiter (takeoff): %f", limiter_);
    }
    if(STATE_HOVER_ && limiter_ >= 1)
    {
      limiter_ = 1;
      ROS_INFO_THROTTLE(1,"command_.F before limiting (STATE_HOVER_): %f", command_.F);
      command_.F = limiter_ * command_.F;
      ROS_INFO_THROTTLE(1,"command_.F after  limiting (STATE_HOVER_): %f", command_.F);
      ROS_INFO_THROTTLE(1,"limiter (takeoff): %f", limiter_);
    }
    if(STATE_LANDING_)
    {
    	limiter_ = limiter_ + landing_slew_rate_;
      ROS_INFO_THROTTLE(1,"command_.F before limiting (STATE_LANDING_): %f", command_.F);
      command_.F = limiter_ * command_.F;
      ROS_INFO_THROTTLE(1,"command_.F after  limiting (STATE_LANDING_): %f", command_.F);
      ROS_INFO_THROTTLE(1,"limiter (takeoff): %f", limiter_);
    }

    ROS_INFO_THROTTLE(1, "STATE_TAKEOFF_: %d", STATE_TAKEOFF_);
    ROS_INFO_THROTTLE(1, "STATE_LANDING_: %d", STATE_LANDING_);
    ROS_INFO_THROTTLE(1, "STATE_LANDED_: %d", STATE_LANDED_);
}

void Controller::publishCommand()
{
  command_.header.stamp = ros::Time::now();
  command_pub_.publish(command_);
}

void Controller::resetIntegrators()
{
  PID_x_dot_.clearIntegrator();
  PID_y_dot_.clearIntegrator();
  PID_z_dot_.clearIntegrator();
  PID_n_.clearIntegrator();
  PID_e_.clearIntegrator();
  PID_d_.clearIntegrator();
  PID_psi_.clearIntegrator();
}

double Controller::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}  // namespace controller
