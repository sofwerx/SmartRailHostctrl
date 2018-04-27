/*
 * =====================================================================================
 *
 *       Filename:  serial_streaming.hpp
 *
 *    Description:  a utility class to support the development of serial tools
 *
 *        Version:  1.0
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

#ifndef __SERIAL_STREAMING_H__
#define __SERIAL_STREAMING_H__

#include <string>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/console.h>

namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;
typedef boost::asio::serial_port Socket;

class StreamSession
{
public:
  StreamSession(boost::asio::io_service& io_service, std::string port, int baud,
    int character_size, bool flow_control, bool parity, int stop_bits)
    : socket_(io_service),
      pulse_timer_(io_service),
      port_(port), baud_(baud),
      character_size_(character_size),
      ros_spin_timer_(io_service),
      flow_control_(flow_control),
      parity_(parity),
      stop_bits_(stop_bits)
   {
    ROS_INFO_STREAM_NAMED("stream_session", "StreamSession configured for " << port_ << " at " << baud << "bps.");
    pulse_interval_ = boost::posix_time::milliseconds(2000);
    ros_spin_interval_ = boost::posix_time::milliseconds(10);
    failed_connection_attempts_ = 0;
    connect();
  }

  void start()
  {
    ROS_DEBUG_NAMED("stream_session", "Starting Streaming Session.");
    active_=true;

    pulse_timer_.expires_from_now(pulse_interval_);
    pulse_timer_.async_wait(boost::bind(&StreamSession::stream_pulse, this,
          boost::asio::placeholders::error));
  }

  void stop()
  {
    ros_callback_queue_.clear();
    pulse_timer_.cancel();

    socket_.close();
    active_=false;
  }

  bool is_active()
  {
    return active_;
  }

private:
  void connect()
  {
    ROS_DEBUG_STREAM_NAMED("StreamSession", "Invoked: CheckConnection");
    if (!is_active())
    {
      ROS_DEBUG_STREAM_NAMED("StreamSession", "Inactive Connection In Check");
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (ros::ok())
    {
      ROS_DEBUG_STREAM_NAMED("StreamSession", "ros::ok() setting spin timer");
      ros_spin_timer_.expires_from_now(ros_spin_interval_);
      ros_spin_timer_.async_wait(boost::bind(&StreamSession::connect, this));
    }
  }

  void attempt_connection()
  {
    ROS_DEBUG("Opening serial port.");

    boost::system::error_code ec;
    socket_.open(port_, ec);
    if (ec) {
      failed_connection_attempts_++;
      if (failed_connection_attempts_ == 1) {
        ROS_ERROR_STREAM("Unable to open port " << port_ << ": " << ec);
      } else {
        ROS_DEBUG_STREAM("Unable to open port " << port_ << " (" << failed_connection_attempts_ << "): " << ec);
      }
      return;
    }
    ROS_INFO_STREAM_NAMED("StreamSession", "Opened " << port_);
    failed_connection_attempts_ = 0;

    typedef boost::asio::serial_port_base serial;
    socket_.set_option(serial::baud_rate(baud_));
    socket_.set_option(serial::character_size(serial::character_size(character_size_)));
    socket_.set_option(serial::stop_bits(serial::stop_bits::one));
    socket_.set_option(serial::parity(serial::parity::none));
    socket_.set_option(serial::flow_control(serial::flow_control::none));
    // Kick off the session.
    start();
  }


  /** Periodically handle ROS callbacks
    */
  void ros_spin_timeout(const boost::system::error_code& error) {
    ros_callback_queue_.callAvailable();

    if (ros::ok())
    {
      ros_spin_timer_.expires_from_now(ros_spin_interval_);
      ros_spin_timer_.async_wait(boost::bind(&StreamSession::ros_spin_timeout, this,
        boost::asio::placeholders::error));
    }
  }

 /** This Implmentation does not receive any messages, it only streams data across
  * its port. It does this at an interval.
  */
  void write_direct_message(Buffer& message) {
    ROS_DEBUG_STREAM_NAMED("StreamSession", "Invoked write_directo_message using a buffer containing "
      << message.size() << " bytes, and of capacity " << message.capacity()<< " bytes");
    uint16_t length = message.size();
    BufferPtr buffer_ptr(new Buffer(length));
    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size());
    stream << (uint8_t)0x02 << (uint8_t)0x03 << (uint8_t)0x00 << (uint8_t)0x0a;
    stream << (uint8_t)0xa7 << (uint8_t)0xa9 << (uint8_t)0x0d << (uint8_t)0xad;
    stream << (uint8_t)0x1d << (uint8_t)0x03;
    // stream << 0x02 << 0x03 << 0x00 << 0x0a << 0xa7 << 0xa9 << 0x0d << 0xad << 0x1d << 0x03;
    ROS_DEBUG_NAMED("async_write", "Sending buffer of %d bytes to client.", length);
    boost::asio::async_write(socket_, boost::asio::buffer(*buffer_ptr),
      boost::bind(&StreamSession::write_completion_cb, this, boost::asio::placeholders::error,
      buffer_ptr));
  }

  //// SENDING Correction MESSAGES ////
  void write_correction_message(Buffer& message) {
    ROS_DEBUG_STREAM_NAMED("StreamSession", "Invoked write_correction_message using a buffer containing "
      << message.size() << " bytes, and of capacity " << message.capacity()<< " bytes");
    uint16_t length = message.size();
    BufferPtr buffer_ptr(new Buffer(length));
    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size());
    stream << (uint8_t)0x02 << (uint16_t)0x03 << (uint8_t)0x15;
    stream << (float)3.1428 << (float)9.789 << (uint32_t)0x01 << (uint32_t)0xad;
    stream << (uint8_t)0x03;
    ROS_DEBUG_NAMED("async_write", "Sending buffer of %d bytes to client.", length);
    boost::asio::async_write(socket_, boost::asio::buffer(*buffer_ptr),
      boost::bind(&StreamSession::write_completion_cb, this, boost::asio::placeholders::error,
      buffer_ptr));
  }


  void write_completion_cb(const boost::system::error_code& error,
                           BufferPtr buffer_ptr) {
    if (error) {
      if (error == boost::system::errc::io_error) {
        ROS_WARN_THROTTLE(1, "Socket write operation returned IO error.");
      } else if (error == boost::system::errc::no_such_device) {
        ROS_WARN_THROTTLE(1, "Socket write operation returned no device.");
      } else {
        ROS_WARN_STREAM_THROTTLE(1, "Unknown error returned during write operation: " << error);
      }
      stop();
    }
    // Buffer is destructed when this function exits and buffer_ptr goes out of scope.
  }

  //// Build Message And Timer ////
  void stream_pulse(const boost::system::error_code& error) {
    ROS_DEBUG("Sending Pulse Message.");
    std::vector<uint8_t> message(21);
    write_correction_message(message);

    pulse_timer_.expires_from_now(pulse_interval_);
    pulse_timer_.async_wait(boost::bind(&StreamSession::stream_pulse, this,
          boost::asio::placeholders::error));
  }

  Socket socket_;
  enum { buffer_max = 1023 };

  ros::NodeHandle nh_;
  ros::CallbackQueue ros_callback_queue_;

  boost::posix_time::time_duration pulse_interval_;
  boost::posix_time::time_duration ros_spin_interval_;
  boost::asio::deadline_timer pulse_timer_;
  boost::asio::deadline_timer ros_spin_timer_;

  bool active_=false;
  std::string port_;
  int baud_;
  int character_size_;
  bool flow_control_;
  bool parity_;
  int stop_bits_;
  int failed_connection_attempts_;
};

}  // namespace

#endif
