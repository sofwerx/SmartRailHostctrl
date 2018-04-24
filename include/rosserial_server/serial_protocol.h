/*
 * =====================================================================================
 *
 *       Filename:  serial_protocol.h
 *
 *    Description:  an abstraction of a serial protocol to be applied within rosserial
 *
 *        Version:  1.0
 *        Created:  04/19/2018 07:53:00 PM
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
#ifndef __SERIAL_PROTOCOL_H__
#define __SERIAL_PROTOCOL_H__

#include <string>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include "rosserial_server/pgs_session.h"

namespace rosserial_server
{

class SerialPgsSession: public PgsSession<boost::asio::serial_port>
{
public:
  SerialPgsSession(boost::asio::io_service& io_service, std::string port, int baud, int character_size,
    bool flow_control, bool parity, int stop_bits)
    : PgsSession(io_service), port_(port), baud_(baud), character_size_(character_size),
        flow_control_(flow_control), parity_(parity), stop_bits_(stop_bits), timer_(io_service)
   {
    ROS_INFO_STREAM_NAMED("pgs_session", "SerialPgsSession configured for " << port_ << " at " << baud << "bps.");
    failed_connection_attempts_ = 0;
    check_connection();
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
      timer_.expires_from_now(boost::posix_time::milliseconds(1000));
      timer_.async_wait(boost::bind(&SerialPgsSession::check_connection, this));
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
    ROS_INFO_STREAM("Opened " << port_);
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

  std::string port_;
  int baud_;
  int character_size_;
  bool flow_control_;
  bool parity_;
  int stop_bits_;
  boost::asio::deadline_timer timer_;
  int failed_connection_attempts_;
};

}  // namespace

#endif
