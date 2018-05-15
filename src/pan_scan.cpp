/*
 * =====================================================================================
 *
 *       Filename:  pan_scan.cpp
 *
 *    Description:  a node executing the action of panning continuously left and right
 *                  through a range of motion
 *
 *        Version:  1.0
 *        Created:  05/15/2018 10:41:10 AM
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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

using geometry_msgs::Twist;
using ros::Publisher;
using ros::Subscriber;
using boost::asio::io_service;
using boost::asio::deadline_timer;
using boost::asio::placeholders::error;
using ros::NodeHandle;
using sensor_msgs::JointState;

// Constants
const float PI = 3.1415927;

class PanWorker
{
  public:
    PanWorker(io_service& io, NodeHandle& nh):
      m_ros_spin_timer_(io)
  { 
    m_pub_rotate_= nh.advertise<Twist>("/ptu/rotate_relative", 128);
    ros::Rate rate(1);
    rate.sleep();

    m_ros_spin_interval_ = boost::posix_time::milliseconds(10);
    nh.setCallbackQueue(&ros_callback_queue_);
    m_ros_spin_timer_.expires_from_now(m_ros_spin_interval_);
    m_ros_spin_timer_.async_wait(boost::bind(&PanWorker::ros_spin_timeout, this,
          boost::asio::placeholders::error));

    // Subscribers : Only subscribe to the most recent instructions
    m_ros_joint_state_listener = nh.subscribe<JointState>("/joint_states", 1, &PanWorker::joint_state_listener, this);
  }

  protected:
    void ros_spin_timeout(const boost::system::error_code& error) 
    {
      ros_callback_queue_.callAvailable();

      if (ros::ok())
      {
        m_ros_spin_timer_.expires_from_now(m_ros_spin_interval_);
        m_ros_spin_timer_.async_wait(boost::bind(&PanWorker::ros_spin_timeout, this, boost::asio::placeholders::error));
      }
    } 
    /* 
     * ===  FUNCTION  ======================================================================
     *         Name:  joint_state_listener
     *  Description:  
     * =====================================================================================
     */
    void joint_state_listener (JointState joint_state )
    {
      ROS_DEBUG_STREAM("Heard this latest joint state alright " <<joint_state );
      JointState old_state = m_current_pos_;
      m_current_pos_ = joint_state;
      if (! m_is_valid_) // have we ever received positional data ?
      { 
        m_initial_pos_ = m_current_pos_;
        m_is_valid_ = true;
        Twist half_twist;
        half_twist.angular.x = SWEEP/2.0;
        m_pub_rotate_.publish(half_twist);
      }		/* -----  end of function joint_state_listener  ----- */
      else
      { // we have previously received positional data, and so we know where we're going
        // perhaps here we should look for a period of static motion
        ROS_DEBUG_STREAM("A positional update"); 
      }
    }

  private:
    boost::posix_time::time_duration m_ros_spin_interval_;
    deadline_timer m_ros_spin_timer_;
    Subscriber m_ros_joint_state_listener;  
    Publisher m_pub_rotate_; 
    const float SWEEP = 1.0; // radians of total sweep
    bool m_is_panning_ = false;
    float m_xmin_ = -1 * PI;
    float m_xmax_ = PI;
    JointState m_current_pos_;
    JointState m_initial_pos_;
    float m_xgoal = 0;
    float m_slack = 0.01;
    bool m_is_valid_ = false;

    ros::CallbackQueue ros_callback_queue_;
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

  PanWorker panner(io, nh_);
  io.run();
  return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
