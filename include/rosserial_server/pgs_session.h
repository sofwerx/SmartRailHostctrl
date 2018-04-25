/**
 *
 *  \file
 *  \brief      Class representing a pgs session between this node and
                a pgs client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
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

namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;
typedef boost::shared_ptr<ros::Publisher> PublisherPtr;

template<typename Socket>
class PgsSession : boost::noncopyable
{
public:
  PgsSession(boost::asio::io_service& io_service)
    : socket_(io_service),
      rx_timer_(io_service),
      require_check_timer_(io_service),
      ros_spin_timer_(io_service),
      async_read_buffer_(socket_, buffer_max,
                         boost::bind(&PgsSession::read_failed, this,
                                     boost::asio::placeholders::error))
  {
    ros_spin_interval_ = boost::posix_time::milliseconds(10);

    // TODO: change the interbyte interval based on baud rate
    interbyte_interval_max_ = boost::posix_time::milliseconds(100);

    nh_.setCallbackQueue(&ros_callback_queue_);

    // Intermittent callback to service ROS callbacks. To avoid polling like this,
    // CallbackQueue could in the future be extended with a scheme to monitor for
    // callbacks on another thread, and then queue them up to be executed on this one.
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
      nh_.advertise<smartrail_hostctrl::PgsDirectControl>("pgs_direct_control", 64);
    publishers_[pgs_directctrl_id_] = pub_direct;

    ros::Publisher pub_correction =
      nh_.advertise<smartrail_hostctrl::PgsCorrection>("pgs_correction", 128);
    publishers_[pgs_correction_id_] = pub_correction;

    active_ = true;

    read_sync_header();
  }

  void stop()
  {
    // Abort any pending ROS callbacks.
    ros_callback_queue_.clear();

    // Abort active session timer callbacks, if present.
    rx_timer_.cancel();
    require_check_timer_.cancel();

    // Reset the state of the session, dropping any publishers or subscribers
    // we currently know about from this client.
    callbacks_.clear();

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
  // TODO: Total message timeout, implement primarily in ReadBuffer.
  void read_sync_header() {
    async_read_buffer_.read(1, boost::bind(&PgsSession::read_start_text, this, _1));
  }

  void read_start_text(ros::serialization::IStream& stream) {
    uint8_t stx;
    stream >> stx;
    if (stx == 0x02) {
      async_read_buffer_.read(3, 1, boost::bind(&PgsSession::read_message_type, this, _1));
    } else {
      read_sync_header();
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
    ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Type Received");
    uint8_t stx, byte_count;
    uint16_t msg_type;
    stream >> stx >> msg_type >> byte_count;
    async_read_buffer_.read(byte_count, 4, boost::bind(&PgsSession::read_message, this, _1));
  }

  void read_message(ros::serialization::IStream& stream)
  {
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
      ROS_DEBUG_STREAM_NAMED("pgs_session", "Read wrapped message ");
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
        read_sync_header();
      }
      // Kickoff next message read.
    }

    // at this point all messages have been handled.  Prepare to read the next incoming message
    read_sync_header();
  }

  void read_failed(const boost::system::error_code& error) {
    if (error == boost::system::errc::no_buffer_space) {
      // No worries. Begin syncing on a new message.
      ROS_WARN("Overrun on receive buffer. Attempting to regain rx sync.");
      read_sync_header();
    } else if (error) {
      // When some other read error has occurred, stop the session, which destroys
      // all known publishers and subscribers.
      ROS_WARN_STREAM("Socket asio error, closing socket: " << error);
      stop();
    }
  }

  //// Message Timeout Watchdog ////
  /// timing between bytes should be minimal, based on socket baud ///
  void set_rx_timeout()
  {
    if (ros::ok())
    {
      rx_timer_.cancel();
      rx_timer_.expires_from_now(interbyte_interval_max_);
      rx_timer_.async_wait(boost::bind(&PgsSession::rx_timeout, this,
        boost::asio::placeholders::error));
    }
  }

   void rx_timeout(const boost::system::error_code& error) {
    if (error != boost::asio::error::operation_aborted) {
      // No problem, just start looking for that header again.
      read_sync_header();
    }
  }

  static uint32_t checksum(ros::serialization::IStream& stream) {
    return fletcher32(stream.getData(), stream.getLength());
  }

  uint16_t pgs_directctrl_id_ = 128;
  uint16_t pgs_correction_id_ = 132;

  Socket socket_;
  AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 2048 };
  bool active_ = false;

  ros::NodeHandle nh_;

  // TODO investigate the use of this callback_queue_ for multithreaded spinning
  ros::CallbackQueue ros_callback_queue_;

  boost::posix_time::time_duration ros_spin_interval_;
  boost::posix_time::time_duration interbyte_interval_max_;

  boost::asio::deadline_timer rx_timer_;
  boost::asio::deadline_timer require_check_timer_;
  boost::asio::deadline_timer ros_spin_timer_;
  std::string require_param_name_;

  std::map<uint16_t, boost::function<void(ros::serialization::IStream&)> > callbacks_;
  std::map<uint16_t, ros::Publisher> publishers_;
};

}  // namespace

#endif  //
