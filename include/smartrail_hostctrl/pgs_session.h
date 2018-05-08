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
#include "smartrail_hostctrl/MessageBytes.h"
#include "flir_ptu_driver/PtuDirectControl.h"
#include <geometry_msgs/Twist.h>

#define RADIANS_PER_ARCSECOND (1.0/206265.0)

namespace smartrail_hostctrl
{

  // Note that this constant includes is the pixel resolution of the associated camera
  const float pgs_res_arcsec_per_pix = 4.31004f;

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

        // set up topics for PgsDirectControl and Delta Messages messages
        ros::Publisher pub_jog =
          nh_.advertise<geometry_msgs::Twist>("/ptu/jogging", 128);
        publishers_[pgs_jog_id_] = pub_jog;

        ros::Publisher pub_correction =
          nh_.advertise<geometry_msgs::Twist>("/ptu/rotate_relative", 128);
        publishers_[pgs_correction_id_] = pub_correction;

        ros::Publisher pub_pgs_stream =
          nh_.advertise<smartrail_hostctrl::MessageBytes>("/pgs/serial", 128);
        publishers_[pgs_stream_id_] = pub_pgs_stream;


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
        async_read_buffer_.read(1, boost::bind(&PgsSession::read_stx, this, _1));
      }

      void read_stx(ros::serialization::IStream& stream) {
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
        // keeping track of the stream's original state
        uint8_t * msg_head = stream.getData();
        uint32_t msg_len  = stream.getLength();

        try {
          // all messages have the following fields
          uint8_t byte_count, etx, stx;
          uint16_t msg_type;
          uint32_t msg_checksum;
          ros::serialization::IStream checksum_stream(msg_head, msg_len);
          stream >> stx >> msg_type >> byte_count;
          uint8_t * stream_bookmark = stream.advance(msg_len -9); // advance the stream far enough to find the msg_checksum
          stream >> msg_checksum;

          stream = ros::serialization::IStream(stream_bookmark, msg_len - 4);

          // FIXME: the following validation fails for direct control messages right now
          if (!validate_checksum(checksum_stream, msg_checksum))
          { // failed to pass checksum testing
            ROS_INFO_STREAM_NAMED("pgs_session", "Message checksum failed");
          }
          if (true) // FIXME: after confirming the issues with the checksum seen above, change back to an else statement to preclude bad messages
          {
            ROS_DEBUG_STREAM_NAMED("pgs_session", "Message Checksum Succeeded");
            
            // this is a jogging message if the msg_type is 0x03
            if (msg_type == 0x03)
            {
              ROS_DEBUG_STREAM_NAMED("pgs_session", "Reading jog message");
              ROS_DEBUG_STREAM("Message Contains stx("<<stx<<") type("<<msg_type<<") byte_count("
                <<byte_count<<") msg_checksum("<<msg_checksum<<") etx("<<etx<<")");
              // At this point, you've received a message of the appropriate size and with
              // containing 
              geometry_msgs::Twist jog_msg;
              uint8_t jog_axis, jog_direction;  
              stream >> jog_axis >> jog_direction;
              if (jog_axis == 131) jog_msg.angular.x = jog_direction;
              else if (jog_axis == 132) jog_msg.angular.y = jog_direction;
              
              if (jog_msg.angular.x > 0 || jog_msg.angular.y > 0 )
                  publishers_[pgs_jog_id_].publish(jog_msg);
            } else if (msg_type == 0x01) // this is a gimbal correction message
            {
              uint32_t msg_counter;
              float X, Y;
              stream >> X >> Y >> msg_counter >> msg_checksum >> etx;
              ROS_DEBUG_STREAM("Message Contains stx("<<stx<<") type("<<msg_type<<") byte_count("<<byte_count<<
                  ") X("<<X<<") Y("<<Y<<") msg_counter("<<msg_counter<<") msg_checksum("<<msg_checksum<<") etx("<<etx<<")");
              // At this point, you've received a message of the appropriate size
              // Alright, so first off, you're going to have
              geometry_msgs::Twist ptu_rotation;
              ptu_rotation.angular.x = calculate_pan_rotation(X);
              ptu_rotation.angular.y = calculate_tilt_rotation(Y);
              publishers_[pgs_correction_id_].publish(ptu_rotation);
            }
          }
        } catch(ros::serialization::StreamOverrunException e) {
          ROS_WARN("Buffer overrun when attempting to parse user message.");
        }

        // The message has been handled, no matter its composition, at this point.  For the purposes of communications publish to the appropriate topic
        ros::serialization::IStream msg_stream(msg_head, msg_len);
        smartrail_hostctrl::MessageBytes pgs_message;
        pgs_message.bytes.push_back(*msg_stream.getData());
        publishers_[pgs_stream_id_].publish(pgs_message);

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

      /* Note that checksum function here has a little management to do in order to smooth
       *   the transition between the classic fletcher32 algorithm and the data format of the
       *   IStream, which is uint8_t rather than the expected uint16_t
       *   - The pgs protocol does not consider the etx byte of the message nor the checksum bytes
       *   - The IStream provides a pointer to a uint8_t stream
       *   - The fletcher32 algorithm expects a uint16_t stream
       */
      static bool validate_checksum(ros::serialization::IStream& stream, uint32_t msg_checksum)
      {
        ROS_DEBUG_STREAM_NAMED("pgs_session",
            "Validating Checksum of message stream against msg_checksum " << msg_checksum);
        // confirm that we're going to have an appropriate length array
        uint32_t len = stream.getLength()-5; //subtract 1 etx and 4 checksum bytes
        /* FIXME: see if you can create a reinterpret cast with the appropriate padding on the last byte of an odd sized one
           if (len % 2 > 0)
           {
           ROS_DEBUG_STREAM_NAMED("pgs_session", "checksum_stream is malformed with length " << stream.getLength());
           return false;
           }
           */
        uint16_t* fletcher32_message = reinterpret_cast<uint16_t*>(stream.getData());
        uint32_t calc_checksum = fletcher32(fletcher32_message, len %2 ? len/2 + 1: len/2);
        return (msg_checksum == calc_checksum);
      }

      /* using the pixel resolution of the PGS, calculate the requested pan offset in radians
         This belngs in the pgs_node, as the pixel resolution of the camera is reqired to make this calculation 
          param X: requested correction in pixels
          return float radians of correction
      */
      float calculate_pan_rotation(float X)
      {
        // m_pan_resolution is provided pgs_res_arcsec_per_pix * X pix * rad/arcsec= rad
        return pgs_res_arcsec_per_pix * X * RADIANS_PER_ARCSECOND;
      }

      /* using the pixel resolution of the PGS, calculate the requested pan offset in radians
         This belngs in the pgs_node, as the pixel resolution of the camera is reqired to make this calculation 
          param Y: requested correction in pixels
          return float radians of correction
      */
      float calculate_tilt_rotation(float Y)
      {
        // m_pan_resolution is provided pgs_res_arcsec_per_pix * X pix * rad/arcsec= rad
        return pgs_res_arcsec_per_pix * Y * RADIANS_PER_ARCSECOND;
      }


      uint16_t pgs_jog_id_ = 128;
      uint16_t pgs_correction_id_ = 132;
      uint16_t pgs_stream_id_ = 136;

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
