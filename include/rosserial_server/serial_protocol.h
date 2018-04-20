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
#include "rosserial_server/session.h"
#include "rosserial_server/serial_protocol.h"

namespace rosserial_server
{
class ProtocolSession: public Session
{
public:
  ProtocolSession(boost::asio::io_service * io_service, SerialProtocol * protocol):
    Session(io_service)
  {}

private:
  void check_connection()
  {
    ROS_DEBUG("checking connection from inside of ProtocolSession");
    if (!is_active())
    {
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (ros::ok())
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&SerialSession::check_connection, this));
    }
  }
};

class SerialProtocol {
public:
};
} // namespace

#endif
