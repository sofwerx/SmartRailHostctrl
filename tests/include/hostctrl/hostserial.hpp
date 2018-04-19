/*
 * =====================================================================================
 *
 *       Filename:  hostserial.hpp
 *
 *    Description:  an interface for a serial library for the host controller
 *
 *        Version:  1.0
 *        Created:  04/16/2018 10:33:07 AM
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

#ifndef __HOSTSERIAL_H__
#define __HOSTSERIAL_H__

#include <boost/asio.hpp>
#include <deque>

using namespace boost;
using namespace std;
using boost::asio::serial_port_base;
using std::string;

// create a proactor object which handles the basics of a serial client
class HostSerial: private boost::noncopyable
{
public:
  HostSerial();

  HostSerial(const string &devname, unsigned int baud_rate, serial_port_base::parity opt_parity=serial_port_base::parity(serial_port_base::parity::none),
    serial_port_base::character_size opt_csize=serial_port_base::character_size(8), serial_port_base::flow_control opt_flow=serial_port_base::flow_control(serial_port_base::flow_control::none),
    serial_port_base::stop_bits opt_stop=serial_port_base::stop_bits(serial_port_base::stop_bits::one));

  void open(const string &devname, unsigned int baud_rate, serial_port_base::parity opt_parity=serial_port_base::parity(serial_port_base::parity::none),
    serial_port_base::character_size opt_csize=serial_port_base::character_size(8), serial_port_base::flow_control opt_flow=serial_port_base::flow_control(serial_port_base::flow_control::none),
    serial_port_base::stop_bits opt_stop=serial_port_base::stop_bits(serial_port_base::stop_bits::one));

  bool isOpen() const;

  bool errorStatus() const;

  void close();

  void write(const char *data, size_t size);

  void write(const std::vector<char>& data);

  void writeString(const std::string& s);

  virtual ~HostSerial()=0;

private:
  static const int max_read_length = 512;

  // wire a callback for the async_read
  void readStart(void);

  // the callback to receive a completed read
  void readComplete(void);

  // all writes to be read
  void doWrite(const char msg);

  // wire the callback starting an asynchronous write
  void writeStart(void);

  // the write callback
  void writeComplete(const boost::system::error_code &error /*e */);

  void doClose();

  // how does this thing fit into the lifecycle of a ros_node ?
  bool active_;
  asio::io_service &io;
  asio::serial_port serial_port_;
  char read_msg_[max_read_length]; // data read from the socket
  deque<char> write_msgs_; // buffered write data
};

#endif
