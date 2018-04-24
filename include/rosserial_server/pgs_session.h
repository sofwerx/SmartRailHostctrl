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
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/Log.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Time.h>
#include "rosserial_server/async_read_buffer.h"
#include "rosserial_server/topic_handlers.h"
#include "fletcher32.h"

namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;

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
    active_ = false;

    timeout_interval_ = boost::posix_time::milliseconds(5000);
    attempt_interval_ = boost::posix_time::milliseconds(1000);
    require_check_interval_ = boost::posix_time::milliseconds(1000);
    ros_spin_interval_ = boost::posix_time::milliseconds(10);
    // TODO: change the interbyte interval based on baud rate
    interbyte_interval_max_ = boost::posix_time::milliseconds(100);

    require_param_name_ = "~require";

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

    callbacks_[rosserial_msgs::TopicInfo::ID_LOG]
        = boost::bind(&PgsSession::handle_log, this, _1);

    // here's the thing, you've got to build a topic message
    // the message
    rosserial_msgs::TopicInfo pgs_topic_info;
    PublisherPtr pub(new Publisher(nh_, topic_info));
    callbacks_[topic_info.topic_id] = boost::bind(&Publisher::handle, pub, _1);
    publishers_[topic_info.topic_id] = pub;


    setup_publisher(pgs_topic_id_);
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
    subscribers_.clear();
    publishers_.clear();
    services_.clear();

    // Close the socket.
    socket_.close();
    active_ = false;
  }

  bool is_active()
  {
    return active_;
  }

  /**
   * This is to set the name of the required topics parameter from the
   * default of ~require. You might want to do this to avoid a conflict
   * with something else in that namespace, or because you're embedding
   * multiple instances of rosserial_server in a single process.
   */
  void set_require_param(std::string param_name)
  {
    require_param_name_ = param_name;
  }

  uint16_t pgs_topic_id_ = 128;

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
      async_read_buffer_.read(20, boost::bind(&PgsSession::read_message, this, stx, _1));
    } else {
      read_sync_header();
    }
  }

  void read_message(uint8_t stx, ros::serialization::IStream& stream) {
    ROS_DEBUG_STREAM_NAMED("pgs_session", "Received message from PGS");
    // is there a smarter way to guarantee that this message gets through with the stx byte
    ros::serialization::IStream checksum_stream(stream.getData()-1, stream.getLength()+1);
    uint32_t msg_checksum = checksum(checksum_stream);

    uint8_t byte_count, etx;
    uint16_t msg_type;
    uint32_t msg_counter, checksum;
    float X, Y;
    stream >> msg_type >> byte_count >> X >> Y >> msg_counter >> checksum >> etx;

    // At this point, you've received a message of the appropriate size
    // Alright, so first off, you're going to have
    if (checksum != msg_checksum) { // passed checksum
      ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Checksum Succeeded");
        try {
          callbacks_[pgs_topic_id_](stream);
        } catch(ros::serialization::StreamOverrunException e) {
            ROS_WARN("Buffer overrun when attempting to parse user message.");
        }
    }
    else {
      ROS_INFO_STREAM_NAMED("pgs_session", "Message checksum failed: calculated (" << msg_checksum << ")!=(" << checksum <<")");
      read_sync_header();
    }
    // Kickoff next message read.
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

  template<typename M>
  bool check_set(std::string param_name, M map) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < param_list.size(); ++i) {
      ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string required_topic((std::string(param_list[i])));
      // Iterate through map of registered topics, to ensure that this one is present.
      bool found = false;
      for (typename M::iterator j = map.begin(); j != map.end(); ++j) {
        if (nh_.resolveName(j->second->get_topic()) ==
            nh_.resolveName(required_topic)) {
          found = true;
          ROS_INFO_STREAM("Verified connection to topic " << required_topic << ", given in parameter " << param_name);
          break;
        }
      }
      if (!found) {
        ROS_WARN_STREAM("Missing connection to topic " << required_topic << ", required by parameter " << param_name);
        return false;
      }
    }
    return true;
  }

  static uint32_t checksum(ros::serialization::IStream& stream) {
    return fletcher32(stream.getData(), stream.getLength());
  }

  //// RECEIVED MESSAGE HANDLERS ////

  void setup_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    PublisherPtr pub(new Publisher(nh_, topic_info));
    callbacks_[topic_info.topic_id] = boost::bind(&Publisher::handle, pub, _1);
    publishers_[topic_info.topic_id] = pub;
  }

  void handle_log(ros::serialization::IStream& stream) {
    rosserial_msgs::Log l;
    ros::serialization::Serializer<rosserial_msgs::Log>::read(stream, l);
    if(l.level == rosserial_msgs::Log::ROSDEBUG) ROS_DEBUG("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::INFO) ROS_INFO("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::WARN) ROS_WARN("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::ERROR) ROS_ERROR("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::FATAL) ROS_FATAL("%s", l.msg.c_str());
  }

  Socket socket_;
  AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 1023 };
  bool active_;

  ros::NodeHandle nh_;
  ros::CallbackQueue ros_callback_queue_;

  boost::posix_time::time_duration timeout_interval_;
  boost::posix_time::time_duration attempt_interval_;
  boost::posix_time::time_duration require_check_interval_;
  boost::posix_time::time_duration ros_spin_interval_;
  boost::posix_time::time_duration interbyte_interval_max_;

  boost::asio::deadline_timer rx_timer_;
  boost::asio::deadline_timer require_check_timer_;
  boost::asio::deadline_timer ros_spin_timer_;
  std::string require_param_name_;

  std::map<uint16_t, boost::function<void(ros::serialization::IStream&)> > callbacks_;
  std::map<uint16_t, PublisherPtr> publishers_;
  std::map<uint16_t, SubscriberPtr> subscribers_;
  std::map<std::string, ServiceClientPtr> services_;
};

}  // namespace

#endif  //
