/*
 * =====================================================================================
 *
 *       Filename:  test_serial.cpp
 *
 *    Description:  a set of tests built around boost's asio support for serial communications
 *
 *        Version:  1.0
 *        Created:  04/16/2018 10:21:59 AM
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

#include "hostctrl/hostserial.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <time.h>

using boost::asio::serial_port;
using boost::asio::serial_port_base;
using boost::asio::async_read;
using boost::asio::async_write;

namespace {

  void asioWriteCallback(const boost::system::error_code& /*e*/, time_t* mark)
  {
    // respond here, confirming that the callback has been called 
    *mark = time(NULL);
  }

  class AsioSerialFixture: public ::testing::Test {
    protected:
      AsioSerialFixture():master_(io_, "/dev/pts/4"), slave_(io_, "/dev/pts/5") {
        master_.set_option(serial_port_base::baud_rate(9600));
        slave_.set_option(serial_port_base::baud_rate(9600));
      }

      void SetUp() {
        mark = time(NULL); // ensure that mark is set to the current time
      }

      void TearDown() {
        master_.close();
        slave_.close();
      }

      ~AsioSerialFixture() {
      }

      boost::asio::io_service io_;
      boost::asio::serial_port master_;
      boost::asio::serial_port slave_;
      time_t mark;
  };

  // Declare a test for the communciations subsystem
  TEST(BoostSerial, support)
  {
    bool supported = false;
#ifdef BOOST_ASIO_HAS_SERIAL_PORT
    supported = true; 
#endif
    ASSERT_TRUE(supported); 
  }

  TEST_F (AsioSerialFixture, isMasterOpen) {
    ASSERT_TRUE(master_.is_open());
  }

  TEST_F (AsioSerialFixture, isSlaveOpen) {
    ASSERT_TRUE(slave_.is_open());
  }

  TEST_F (AsioSerialFixture, compMasterSlaveOptions) {    
    // Note tha the serial port options are easily retrieved using the get_option method
    serial_port_base::baud_rate mb = serial_port_base::baud_rate();
    master_.get_option( mb );

    serial_port_base::baud_rate sb = serial_port_base::baud_rate();
    slave_.get_option( sb );
    ASSERT_EQ( mb.value(), sb.value() );
  }

  TEST_F ( AsioSerialFixture, AsyncEcho ) {
     
  }
} // namespace 
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  entry point testing the asio serial interface presented by hostserial 
 * =====================================================================================
 */
int main ( int argc, char *argv[] )
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}				/* ----------  end of function main  ---------- */
