/*!
 * \jaco_joy_teleop.cpp
 * \brief Allows for control of the jaco arm with a joystick.
 *
 * jaco_joy_teleop creates a ROS node that allows for the control of the
 * JACO arm with a joystick. This node listens to a /joy topic.
 *
 * \author David Kent, GT - dekent@gatech.edu
 * \date February 28, 2020
 */

#include <jaco_teleop/jaco_joy_teleop.h>

using namespace std;

jaco_joy_teleop::jaco_joy_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  loadParameters(node);

  // create the ROS topics
  cartesian_cmd = node.advertise<kinova_msgs::PoseVelocity>(arm_name_ + "_driver/in/cartesian_velocity", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &jaco_joy_teleop::joy_cback, this);

//  eStopClient = node.serviceClient<wpi_jaco_msgs::EStop>(topic_prefix_ + "_arm/software_estop");
  stopClient = node.serviceClient<kinova_msgs::Stop>(arm_name_ + "_driver/in/stop");
  startClient = node.serviceClient<kinova_msgs::Start>(arm_name_ + "_driver/in/start");

  if (!kinova_gripper_)
  {
    gripperClient = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_topic_);
  }
  else
  {
    ROS_INFO("Kinova gripper currently not supported, gripper control will be disabled.");
  }

  // read in throttle values
  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);
  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);
  string str;
  private_nh.param<string>("controller_type", str, "digital");
  if (str.compare("digital") == 0)
    controllerType = DIGITAL;
  else
    controllerType = ANALOG;

  //initialize everything
  stopMessageSentArm = true;
  stopMessageSentFinger = true;
  EStopEnabled = false;
  helpDisplayed = false;
  mode = ARM_CONTROL;

  open_sent = false;
  close_sent = false;

  ROS_INFO("Joystick teleop started for: %s", arm_name_.c_str());

  puts(" ----------------------------------------");
  puts("| Joystick Teleop Help                   |");
  puts("|----------------------------------------|*");
  puts("| Current Mode: Arm Control              |*");
  puts("|----------------------------------------|*");
  puts("| For help and controls, press:          |*");
  puts("|                          _             |*");
  puts("|                        _| |_           |*");
  puts("|                       |_   _|          |*");
  puts("|                         |_|            |*");
  puts("|                  show arm controls     |*");
  puts("|                                        |*");
  puts(" ----------------------------------------**");
  puts("  *****************************************");

  if (controllerType == ANALOG)
  {
    initLeftTrigger = false;
    initRightTrigger = false;
    calibrated = false;

    ROS_INFO("You specified a controller with analog triggers. This requires calibration before any teleoperation can begin.  Please press and release both triggers before continuing.");
  }
  else
    calibrated = true;
}

void jaco_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // make sure triggers are calibrated before continuint if an analog controller was specified
  if (!calibrated)
  {
    if (!initLeftTrigger && joy->axes.at(2) == 1.0)
      initLeftTrigger = true;

    if (!initRightTrigger && joy->axes.at(5) == 1.0)
      initRightTrigger = true;

    if (initLeftTrigger && initRightTrigger)
    {
      calibrated = true;
      ROS_INFO("Controller calibration complete!");
    }

    return;
  }

  //software emergency stop
  if ((controllerType == DIGITAL && joy->buttons.at(8) == 1) || (controllerType == ANALOG && joy->buttons.at(6) == 1))
  {
    EStopEnabled = true;
    kinova_msgs::Stop stop_srv;
    if (!stopClient.call(stop_srv))
      ROS_INFO("Couldn't call software estop service.");
  }
  else if ((controllerType == DIGITAL && joy->buttons.at(9) == 1) || (controllerType == ANALOG && joy->buttons.at(7) == 1))
  {
    EStopEnabled = false;
    kinova_msgs::Start start_srv;
    if (!startClient.call(start_srv))
      ROS_INFO("Couldn't call software estop service.");
  }

  //help menu
  if ((controllerType == DIGITAL && joy->axes.at(5) == -1.0) || (controllerType == ANALOG && joy->axes.at(7) == -1.0))
  {
    if (!helpDisplayed)
    {
      helpDisplayed = true;
      puts(" ----------------------------------------");
      puts("| Joystick Teleop Help                   |");
      puts("|----------------------------------------|*");
      if (mode == ARM_CONTROL)
        puts("| Current Mode: Arm Control              |*");
      else
        puts("| Current Mode: Finger Control           |*");
      puts("|----------------------------------------|*");
      puts("|                Controls                |*");
      puts("|   roll/down                 roll/up    |*");
      puts("|    ________                ________    |*");
      puts("|   /    _   \\______________/        \\   |*");
      puts("|  |   _| |_    < >    < >     (4)    |  |*");
      puts("|  |  |_   _|  estop  start (1)   (3) |  |*");
      puts("|  |    |_|    ___      ___    (2)    |  |*");
      puts("|  |          /   \\    /   \\          |  |*");
      puts("|  |          \\___/    \\___/          |  |*");
      puts("|  |       x/y trans  pitch/yaw       |  |*");
      puts("|  |        _______/--\\_______        |  |*");
      puts("|  |       |                  |       |  |*");
      puts("|   \\     /                    \\     /   |*");
      puts("|    \\___/                      \\___/    |*");
      puts("|                                        |*");
      puts("| Buttons:                               |*");
      puts("|   (1) No function                      |*");
      puts("|   (2) Close gripper                    |*");
      puts("|   (3) Open gripper                     |*");
      puts("|   (4) No function                      |*");
      puts(" ----------------------------------------**");
      puts("  *****************************************");
    }
  }
  else
    helpDisplayed = false;

  int buttonIndex1, buttonIndex2;

  switch (mode)
  {
    case ARM_CONTROL:
      // left joystick controls the linear x and y movement
      cartesianCmd.twist_linear_x = joy->axes.at(0) * MAX_TRANS_VEL * linear_throttle_factor;
      cartesianCmd.twist_linear_y = -joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;

      //triggers control the linear z movement
      if (controllerType == DIGITAL)
      {
        if (joy->buttons.at(7) == 1)
          cartesianCmd.twist_linear_z = MAX_TRANS_VEL * linear_throttle_factor;
        else if (joy->buttons.at(6) == 1)
          cartesianCmd.twist_linear_z = -MAX_TRANS_VEL * linear_throttle_factor;
        else
          cartesianCmd.twist_linear_z = 0.0;
      }
      else
      {
        if (joy->axes.at(5) < 1.0)
          cartesianCmd.twist_linear_z = (0.5 - joy->axes.at(5) / 2.0) * MAX_ANG_VEL * angular_throttle_factor;
        else
          cartesianCmd.twist_linear_z = -(0.5 - joy->axes.at(2) / 2.0) * MAX_ANG_VEL * angular_throttle_factor;
      }

      //bumpers control roll
      if (joy->buttons.at(5) == 1)
        cartesianCmd.twist_angular_z = MAX_ANG_VEL * angular_throttle_factor;
      else if (joy->buttons.at(4) == 1)
        cartesianCmd.twist_angular_z = -MAX_ANG_VEL * angular_throttle_factor;
      else
        cartesianCmd.twist_angular_z = 0.0;

      //right joystick controls pitch and yaw
      if (controllerType == DIGITAL)
      {
        cartesianCmd.twist_angular_x = -joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
        cartesianCmd.twist_angular_y = joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor;
      }
      else
      {
        cartesianCmd.twist_angular_x = -joy->axes.at(4) * MAX_ANG_VEL * angular_throttle_factor;
        cartesianCmd.twist_angular_y = joy->axes.at(3) * MAX_ANG_VEL * angular_throttle_factor;
      }

      //mode switching (not used, since we've just implemented open/close functionality)
      if (controllerType == DIGITAL)
      {
        buttonIndex1 = 2;
        buttonIndex2 = 3;
      }
      else
      {
        buttonIndex1 = 0;
        buttonIndex2 = 1;
      }

      if (joy->buttons.at(buttonIndex1) == 1)  // close gripper
      {
        open_sent = false;
        if (!close_sent)
        {
          gripperClient->cancelAllGoals();
          control_msgs::GripperCommandGoal gripper_goal;
          gripper_goal.command.max_effort = gripper_max_effort_;
          gripper_goal.command.position = gripper_closed_pos_;
          gripperClient->sendGoal(gripper_goal);
          close_sent = true;
        }
      }
      else if (joy->buttons.at(buttonIndex2) == 1)  // open gripper
      {
        close_sent = false;
        if (!open_sent)
        {
          gripperClient->cancelAllGoals();
          control_msgs::GripperCommandGoal gripper_goal;
          gripper_goal.command.max_effort = gripper_max_effort_;
          gripper_goal.command.position = gripper_open_pos_;
          gripperClient->sendGoal(gripper_goal);
          open_sent = true;
        }
      }
      else
      {
        open_sent = false;
        close_sent = false;
      }
      break;
  }
}

void jaco_joy_teleop::publish_velocity()
{
  //publish stop commands if EStop is enabled
  if (EStopEnabled)
    return;

  switch (mode)
  {
    case ARM_CONTROL:
      //only publish stop message once; this allows other nodes to publish velocities
      //while the controller is not being used
      if (cartesianCmd.twist_linear_x == 0.0 && cartesianCmd.twist_linear_y == 0.0 && cartesianCmd.twist_linear_z == 0.0
          && cartesianCmd.twist_angular_x == 0.0 && cartesianCmd.twist_angular_y == 0.0
          && cartesianCmd.twist_angular_z == 0.0)
      {
        if (!stopMessageSentArm)
        {
          cartesian_cmd.publish(cartesianCmd);
          stopMessageSentArm = true;
        }
      }
      else
      {
        // send the twist command
        cartesian_cmd.publish(cartesianCmd);
        stopMessageSentArm = false;
      }
      break;
  }
}

bool jaco_joy_teleop::loadParameters(const ros::NodeHandle n)
{
  n.param("jaco_joy_teleop/arm_name", arm_name_, std::string("j2s7s300"));
  n.param("jaco_joy_teleop/kinova_gripper", kinova_gripper_, false);
  n.param("jaco_joy_teleop/gripper_topic", gripper_topic_, std::string("/gripper_actions/gripper_command"));
  n.param("jaco_joy_teleop/gripper_closed_pos", gripper_closed_pos_, 0.0);
  n.param("jaco_joy_teleop/gripper_open_pos", gripper_open_pos_, 0.085);
  n.param("jaco_joy_teleop/gripper_max_effort", gripper_max_effort_, 200.0);

  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "jaco_joy_teleop");

  // initialize the joystick controller
  jaco_joy_teleop controller;

  ros::Rate loop_rate(100);  //rate at which to publish velocity commands (100 Hz required for kinova_ros package)
  while (ros::ok())
  {
    controller.publish_velocity();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
