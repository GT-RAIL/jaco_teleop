/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Willow Garage, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \jaco_key_teleop.h
 * \brief Allows for control of the JACO arm with a keyboard.
 *
 * jaco_key_teleop creates a ROS node that allows the control of the JACO arm
 * with a keyboard.
 *
 * \author David Kent, GT - dekent@gatech.edu
 * \date February 28, 2020
 */

#include <jaco_teleop/jaco_key_teleop.h>

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

jaco_key_teleop::jaco_key_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  loadParameters(nh_);

  // create the ROS topics
  cartesian_cmd = nh_.advertise<kinova_msgs::PoseVelocity>(arm_name_ + "_driver/in/cartesian_velocity", 10);

  // read in throttle values
  double temp;
  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);
  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);
  mode = ARM_CONTROL;

  ROS_INFO("Keyboard teleop started for %s", arm_name_.c_str());
}

void jaco_key_teleop::watchdog()
{
  switch (mode)
  {
    case ARM_CONTROL:
    {
      boost::mutex::scoped_lock lock(publish_mutex_);
      if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))
          && (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
      {
        kinova_msgs::PoseVelocity cmd;
        cmd.twist_linear_x = 0.0;
        cmd.twist_linear_y = 0.0;
        cmd.twist_linear_z = 0.0;
        cmd.twist_angular_x = 0.0;
        cmd.twist_angular_y = 0.0;
        cmd.twist_angular_z = 0.0;
        cartesian_cmd.publish(cmd);
      }
    }
      break;
    case FINGER_CONTROL:
    {
        // TODO: this is currently not supported
    }
      break;
  }
}

void jaco_key_teleop::loop()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("    Reading from Keyboard    ");
  puts("-----------------------------");
  puts("   Press the H key for help");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

    //Display help message
    if (c == KEYCODE_H)
    {
      displayHelp();
    }

    switch (mode)
    {
      case ARM_CONTROL:
      {
        //initialize twist command
        kinova_msgs::PoseVelocity cmd;
        cmd.twist_linear_x = 0.0;
        cmd.twist_linear_y = 0.0;
        cmd.twist_linear_z = 0.0;
        cmd.twist_angular_x = 0.0;
        cmd.twist_angular_y = 0.0;
        cmd.twist_angular_z = 0.0;

        // w/s control forward/backward translation
        // a/d control left/right translation
        // r/f control up/down translation
        // q/e controls roll
        // up/down controls pitch
        // left/right controls yaw
        switch (c)
        {
          case KEYCODE_W:
            cmd.twist_linear_y = -MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_S:
            cmd.twist_linear_y = MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_A:
            cmd.twist_linear_x = MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_D:
            cmd.twist_linear_x = -MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_R:
            cmd.twist_linear_z = MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_F:
            cmd.twist_linear_z = -MAX_TRANS_VEL * linear_throttle_factor;
            break;
          case KEYCODE_Q:
            cmd.twist_angular_z = -MAX_ANG_VEL * angular_throttle_factor;
            break;
          case KEYCODE_E:
            cmd.twist_angular_z = MAX_ANG_VEL * angular_throttle_factor;
            break;
          case KEYCODE_UP:
            cmd.twist_angular_x = -MAX_ANG_VEL * angular_throttle_factor;
            break;
          case KEYCODE_DOWN:
            cmd.twist_angular_x = MAX_ANG_VEL * angular_throttle_factor;
            break;
          case KEYCODE_LEFT:
            cmd.twist_angular_y = MAX_ANG_VEL * angular_throttle_factor;
            break;
          case KEYCODE_RIGHT:
            cmd.twist_angular_y = -MAX_ANG_VEL * angular_throttle_factor;
            break;
          case
          KEYCODE_2:
            mode = FINGER_CONTROL;
            ROS_INFO("Activated finger control mode");
            break;
        }

        //publish twist to arm controller
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
        {
          first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        cartesian_cmd.publish(cmd);
      }
        break;
      case FINGER_CONTROL:
      {
        // TODO: this is currently not supported
        switch (c)
        {
          case KEYCODE_1:
            mode = ARM_CONTROL;
            ROS_INFO("Activated arm control mode");
            break;
        }
      }
        break;
    }
  }
}

void jaco_key_teleop::displayHelp()
{
  switch (mode)
  {
    case ARM_CONTROL:
      puts(" ------------------------------------");
      puts("| Jaco Keyboard Teleop Help          |");
      puts("|------------------------------------|*");
      puts("| Current Mode: Arm Control          |*");
      puts("|------------------------------------|*");
      puts("| w/s : forward/backward translation |*");
      puts("| a/d : left/right translation       |*");
      puts("| r/f : up/down translation          |*");
      puts("| q/e : roll                         |*");
      puts("| up/down : pitch                    |*");
      puts("| left/right : yaw                   |*");
      puts("| 2 : switch to Finger Control       |*");
      puts(" ------------------------------------**");
      puts("  *************************************");
      break;
    case FINGER_CONTROL:
      puts(" ------------------------------------");
      puts("| Jaco Keyboard Teleop Help          |");
      puts("|------------------------------------|*");
      puts("| Current Mode: Finger Control       |*");
      puts("|------------------------------------|*");
      puts("| (this mode is currently not        |*");
      puts("|  supported!)                       |*");
      puts("| 1 : switch to Arm Control          |*");
      puts(" ------------------------------------**");
      puts("  *************************************");
      break;
  }
}

bool jaco_key_teleop::loadParameters(const ros::NodeHandle n)
{
  n.param("jaco_key_teleop/arm_name", arm_name_, std::string("j2s7s300"));

  //! @todo MdL [IMPR]: Return is values are all correctly loaded.
  return true;
}

void shutdown(int sig)
{
  // shut everything down
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "jaco_key_teleop");

  // initialize the keyboard controller
  jaco_key_teleop key_controller;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread my_thread(boost::bind(&jaco_key_teleop::loop, &key_controller));
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&jaco_key_teleop::watchdog, &key_controller));
  ros::spin();

  // wait for everything to end
  my_thread.interrupt();
  my_thread.join();

  return EXIT_SUCCESS;
}
