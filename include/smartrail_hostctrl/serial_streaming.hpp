/*
 * =====================================================================================
 *
 *       Filename:  serial_streaming.hpp
 *
 *    Description:  a utility class to support the development of serial tools
 *
 *        Version:  1.0
 *        Created:  04/23/2018 06:54:03 PM
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
    : socket_(io_service), pulse_timer_(io_service), port_(port), baud_(baud), 
      character_size_(character_size), ros_spin_timer_(io_service), flow_control_(flow_control), 
      parity_(parity), stop_bits_(stop_bits)
   {
    ROS_INFO_STREAM_NAMED("stream_session", "StreamSession configured for " << port_ << " at " << baud << "bps.");
    failed_connection_attempts_ = 0;
    check_connection();
  }

  Socket& socket()
  {
    return socket_;
  }

  void start()
  {
    ROS_DEBUG_NAMED("stream_session", "Starting Streaming Session.");
    active_=true;
    // TODO: start the pulse
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
  void check_connection()
  {
    if (!is_active())
    {
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (ros::ok())
    {
      ros_spin_timer_.expires_from_now(boost::posix_time::milliseconds(1000));
      ros_spin_timer_.async_wait(boost::bind(&StreamSession::check_connection, this));
    }
  }

  void attempt_connection()
  {
    ROS_DEBUG("Opening serial port.");

    boost::system::error_code ec;
    socket().open(port_, ec);
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
    socket().set_option(serial::baud_rate(baud_));
    socket().set_option(serial::character_size(serial::character_size(character_size_)));
    socket().set_option(serial::stop_bits(serial::stop_bits::one));
    socket().set_option(serial::parity(serial::parity::none));
    socket().set_option(serial::flow_control(serial::flow_control::none));

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

  //// SENDING MESSAGES ////
  void write_message(Buffer& message) {
    uint8_t overhead_bytes = 8;
    uint16_t length = overhead_bytes + message.size();
    BufferPtr buffer_ptr(new Buffer(length));

    uint8_t msg_checksum;
    ros::serialization::IStream checksum_stream(message.size() > 0 ? &message[0] : NULL, message.size());

    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size());
    /*
    uint8_t msg_len_checksum = 255 - checksum(message.size());
    stream << (uint16_t)0xfeff << (uint16_t)message.size() << msg_len_checksum << topic_id;
    msg_checksum = 255 - (checksum(checksum_stream) + checksum(topic_id));

    memcpy(stream.advance(message.size()), &message[0], message.size());
    stream << msg_checksum;
    */
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
    std::vector<uint8_t> message(0);
    ROS_DEBUG("Sending Pulse Message.");
    write_message(message);

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

  bool active_;
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
