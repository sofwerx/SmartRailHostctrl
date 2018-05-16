/*
 * =====================================================================================
 *
 *       Filename:  tilt_scan.cpp
 *
 *    Description:  tilting continously in a controlled manner.  Intended as a utlity
 *                  to be invoked via roslaunch.
 *
 *        Version:  1.0
 *        Created:  05/15/2018 10:49:49 AM
 *       Revision:  none
 *       Compiler:  gcc
 *        License:  MIT
 *         Author:  Gary Hendrick gary.hendrick.fellow@sofwerx.org
 *   Organization:  SOFWerx
 *
 * =====================================================================================
 *  Copyright 2018 Gary Hendrick
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the "Software"), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *  of the Software, and to permit persons to whom the Software is furnished to do
 *  so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#include <stdlib.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>

using ros::Publisher;
using ros::Subscriber;
using boost::asio::io_service;
using boost::asio::deadline_timer;
using boost::asio::placeholders::error;
using ros::NodeHandle;
using sensor_msgs::JointState;

// Constants
const float PI = 3.1415927;

class TiltWorker
{
  public:
    TiltWorker(io_service& io, NodeHandle& nh):
      m_ros_spin_timer(io)
  { 
    m_pub_rotate= nh.advertise<JointState>("/ptu/cmd", 4);
    ros::Rate rate(1);
    rate.sleep();

    m_ros_spin_interval = boost::posix_time::milliseconds(10);
    nh.setCallbackQueue(&ros_callback_queue);
    m_ros_spin_timer.expires_from_now(m_ros_spin_interval);
    m_ros_spin_timer.async_wait(boost::bind(&TiltWorker::ros_spin_timeout, this,
          boost::asio::placeholders::error));

    // Subscribers : Only subscribe to the most recent instructions
    m_ros_joint_state_listener = nh.subscribe<JointState>("/joint_states", 1, &TiltWorker::joint_state_listener, this);

    build_pos(m_down_position);
    build_pos(m_up_position);
    build_pos(m_initial_pos);
  }

  protected:
    void build_pos(JointState& js)
    {
      js.header.stamp = ros::Time::now();
      js.name.resize(2);
      js.position.resize(2);
      js.velocity.resize(2);
      js.name[0] = "pan";
      js.name[1] = "tilt";
    }

    void ros_spin_timeout(const boost::system::error_code& error) 
    {
      ros_callback_queue.callAvailable();

      if (ros::ok())
      {
        m_ros_spin_timer.expires_from_now(m_ros_spin_interval);
        m_ros_spin_timer.async_wait(boost::bind(&TiltWorker::ros_spin_timeout, this, boost::asio::placeholders::error));
      }
    } 
    /* 
     * ===  FUNCTION  ======================================================================
     *         Name:  joint_state_listener
     *  Description:  this listener allows us to use joint state positioning to 
     *                determine if the system is at a desired location, allowing the
     *                chaining of locations.
     * =====================================================================================
     */
    void joint_state_listener (JointState joint_state )
    {
      ROS_DEBUG_STREAM("Heard this latest joint state alright " <<joint_state );
      JointState old_state = m_current_pos;
      m_current_pos = joint_state;
      if (! m_is_valid) // have we ever received positional data ?
      { 
        m_initial_pos = m_current_pos;
        m_down_position.position[0] = m_current_pos.position[0];
        m_down_position.position[1] = m_current_pos.position[1] - SWEEP/2;
        m_down_position.velocity[0] = m_current_pos.velocity[0];
        m_down_position.velocity[1] = m_current_pos.velocity[1];
        m_up_position.position[0] = m_current_pos.position[0];
        m_up_position.position[1] = m_current_pos.position[1] + SWEEP/2;
        m_up_position.velocity[0] = m_current_pos.velocity[0];
        m_up_position.velocity[1] = m_current_pos.velocity[1];
        m_is_valid = true;
        pan(m_down_position);
      }		/* -----  end of function joint_state_listener  ----- */
      else
      { // we have previously received positional data, and so we know where we're going
        // perhaps here we should look for a period of static motion
        if (abs(m_down_position.position[1] - m_current_pos.position[1]) < 0.1) pan(m_up_position);
        if (abs(m_up_position.position[1] - m_current_pos.position[1]) < 0.1) pan(m_down_position);
      }
    }

    void pan(JointState cmd_state) 
    {
      m_pub_rotate.publish(cmd_state);
    }

  private:
    boost::posix_time::time_duration m_ros_spin_interval;
    deadline_timer m_ros_spin_timer;
    Subscriber m_ros_joint_state_listener;  
    Publisher m_pub_rotate; 
    const float SWEEP = 1.0; // radians of total sweep
    bool m_is_tilting = false;
    float m_xmin_ = -1 * PI;
    float m_xmax_ = PI;
    JointState m_current_pos;
    JointState m_initial_pos;
    JointState m_down_position;
    JointState m_up_position;
    float m_xgoal = 0;
    float m_slack = 0.01;
    bool m_is_valid = false;

    ros::CallbackQueue ros_callback_queue;
};

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  entry point for pan_scan
 * =====================================================================================
 */
int main ( int argc, char *argv[] )
{
  ros::init(argc, argv, "pan_scan");  
  ros::NodeHandle nh_;
  io_service io;

  TiltWorker panner(io, nh_);
  io.run();
  return EXIT_SUCCESS;
}
