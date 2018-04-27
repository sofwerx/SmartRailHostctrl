/*
 * =====================================================================================
 *
 *       Filename:  pgs_session.h
 *
 *    Description:  a PGS Session Protocol
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
 *  SOFTWARE.*
 */

#ifndef __PGS_SESSION_H__
#define __PGS_SESSION_H__

#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Time.h>
#include "rosserial_server/async_read_buffer.h"
#include "fletcher32.h"
#include "smartrail_hostctrl/PgsDirectControl.h"
#include "smartrail_hostctrl/PgsCorrection.h"

namespace smartrail_hostctrl
{

template<typename Socket>
class PgsSession : boost::noncopyable
{
public:
  PgsSession(boost::asio::io_service& io_service)
    : socket_(io_service),
      ros_spin_timer_(io_service),
      async_read_buffer_(socket_, buffer_max,
                         boost::bind(&PgsSession::read_failed, this,
                                     boost::asio::placeholders::error))
  {
    ros_spin_interval_ = boost::posix_time::milliseconds(10);
    nh_.setCallbackQueue(&ros_callback_queue_);

    ros_spin_timer_.expires_from_now(ros_spin_interval_);
    ros_spin_timer_.async_wait(boost::bind(&PgsSession::ros_spin_timeout, this,
                                           boost::asio::placeholders::error));
  }

  Socket& socket()
  {
    return socket_;
  }

  void start()
  {
    ROS_DEBUG_NAMED("pgs_session", "Starting session for PGS protocol");

    // set up topics for PgsDirectControl and PgsCorrection messages
    ros::Publisher pub_direct =
      nh_.advertise<smartrail_hostctrl::PgsDirectControl>("pgs_direct_control", 128);
    publishers_[pgs_directctrl_id_] = pub_direct;

    ros::Publisher pub_correction =
      nh_.advertise<smartrail_hostctrl::PgsCorrection>("pgs_correction", 128);
    publishers_[pgs_correction_id_] = pub_correction;

    active_ = true;

    read_start_byte();
  }

  void stop()
  {
    // Abort any pending ROS callbacks.
    ros_callback_queue_.clear();

    // Close the socket.
    socket_.close();
    active_ = false;
  }

  bool is_active()
  {
    return active_;
  }

  private:
  /**
   * Periodic function which handles calling ROS callbacks, executed on the same
   * io_service thread to avoid a concurrency nightmare.
   */
  void ros_spin_timeout(const boost::system::error_code& error) {
    ros_callback_queue_.callAvailable();

    if (ros::ok())
    {
      // Call again next interval.
      ros_spin_timer_.expires_from_now(ros_spin_interval_);
      ros_spin_timer_.async_wait(boost::bind(&PgsSession::ros_spin_timeout, this,
                                             boost::asio::placeholders::error));
    }
  }

  //// RECEIVING MESSAGES ////
  void read_start_byte() {
    async_read_buffer_.read(1, boost::bind(&PgsSession::read_start_text, this, _1));
  }

  void read_start_text(ros::serialization::IStream& stream) {
    uint8_t stx;
    stream >> stx;
    if (stx == 0x02) {
      async_read_buffer_.read(3, 1, boost::bind(&PgsSession::read_message_type, this, _1));
    } else {
      read_start_byte();
    }
  }

  void read_message_type(ros::serialization::IStream& stream) {
    ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Type Received");
    uint8_t stx;
    uint16_t msg_type;
    // if there were more to do with the type of message, you'd do it now, but it is time
    // to move on to the byte count
    stream >> stx >> msg_type;
    async_read_buffer_.read(4, 3, boost::bind(&PgsSession::read_byte_count, this, _1));
  }

   void read_byte_count(ros::serialization::IStream& stream) {
    ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Byte Count Received");
    uint8_t stx, byte_count;
    uint16_t msg_type;
    stream >> stx >> msg_type >> byte_count;
    async_read_buffer_.read(byte_count, 4, boost::bind(&PgsSession::read_message, this, _1));
  }

  void read_message(ros::serialization::IStream& stream)
  {
    ROS_DEBUG_STREAM_NAMED("pgs_session", "Message received");
    // all messages have the following fields
    uint8_t byte_count, etx, stx;
    uint16_t msg_type;
    stream >> stx >> msg_type >> byte_count;

    // when byte_count is not identical to the byte_count for gimbal correction messages, this is
    // instead a gimbal control message wrapped in a pgs data frame
    if (byte_count != 21) // TODO: determine if you can use msg_type here rather than byte_count
    {
      uint32_t payload_length = byte_count - 4; //stx (1) + msg_type (2) + byte_count (1)k
      //  Having received a wrapped message, you'll want to push it forward
      ros::serialization::IStream payload(stream.getData(), payload_length);
      ROS_DEBUG_STREAM_NAMED("pgs_session", "Wrapped Message to be streamed via topic");
      try {
        smartrail_hostctrl::PgsDirectControl msg_direct;
        msg_direct.command.push_back(*payload.getData());
        msg_direct.length = payload.getLength();
        publishers_[pgs_directctrl_id_].publish(msg_direct);
        } catch (ros::serialization::StreamOverrunException e) {
          ROS_WARN("Buffer overrun when attempting to parse direct control message.");
        }
    } else // this is a gimbal correction message and requires additional decomposition
    {
      // only gimbal correction messages have the X, Y, Message Counter, and Checksum fields
      ros::serialization::IStream checksum_stream(stream.getData(), stream.getLength());
      uint32_t msg_counter, msg_checksum;
      uint32_t calculated_checksum = checksum(checksum_stream);
      float X, Y;
      stream >> X >> Y >> msg_counter >> msg_checksum >> etx;
      ROS_DEBUG_STREAM("Message Contains stx("<<stx<<") type("<<msg_type<<") byte_count("<<byte_count<<
      ") X("<<X<<") Y("<<Y<<") msg_counter("<<msg_counter<<") msg_checksum("<<msg_checksum<<") etx("<<etx<<")");
      // At this point, you've received a message of the appropriate size
      // Alright, so first off, you're going to have
      if (calculated_checksum == msg_checksum) { // passed checksum
        ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Checksum Succeeded");
          try {
            smartrail_hostctrl::PgsCorrection msg_correction;
            msg_correction.X = X;
            msg_correction.Y=Y;
            publishers_[pgs_correction_id_].publish(msg_correction);
          } catch(ros::serialization::StreamOverrunException e) {
              ROS_WARN("Buffer overrun when attempting to parse user message.");
          }
      }
      else {
        ROS_INFO_STREAM_NAMED("pgs_session", "Message checksum failed: calculated (" << msg_checksum << ")!=(" << checksum <<")");
      }
      // Kickoff next message read.
    }

    // at this point all messages have been handled.  Prepare to read the next incoming message
    read_start_byte();
  }

  void read_failed(const boost::system::error_code& error) {
    if (error == boost::system::errc::no_buffer_space) {
      // No worries. Begin syncing on a new message.
      ROS_WARN("Overrun on receive buffer. Attempting to regain rx sync.");
      read_start_byte();
    } else if (error) {
      // When some other read error has occurred, stop the session, which destroys
      // all known publishers and subscribers.
      ROS_WARN_STREAM("Socket asio error, closing socket: " << error);
      stop();
    }
  }
  
  static uint32_t checksum(ros::serialization::IStream& stream) {
    return fletcher32(stream.getData(), stream.getLength());
  }

  uint16_t pgs_directctrl_id_ = 128;
  uint16_t pgs_correction_id_ = 132;

  Socket socket_;
  
  rosserial_server::AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 2048 };
  bool active_ = false;

  ros::NodeHandle nh_;

  ros::CallbackQueue ros_callback_queue_;

  boost::posix_time::time_duration ros_spin_interval_;
  boost::asio::deadline_timer ros_spin_timer_;
  std::map<uint16_t, ros::Publisher> publishers_;
};

}  // namespace

#endif  //
