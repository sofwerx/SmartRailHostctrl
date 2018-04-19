/*
 * =====================================================================================
 *
 *       Filename:  test_timer.cpp
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

namespace {

  void asioTimeMarkCallback(const boost::system::error_code& /*e*/, time_t* mark)
  {
    // respond here, confirming that the callback has been called 
    *mark = time(NULL);
  }

  class AsioTimerFixture: public ::testing::Test {
    protected:
      void SetUp() {
        is_called = false;
        mark = time(NULL); // ensure that mark is set to the current time
      }

      void TearDown() {
      }

      ~AsioTimerFixture() {
      }

      boost::asio::io_service io;
      bool is_called;
      time_t mark;
  };

  TEST_F (AsioTimerFixture, timerExample1) {
    time_t timer;
    time(&timer); 
    boost::asio::deadline_timer t(io, boost::posix_time::seconds(1));    
    t.wait();
    ASSERT_EQ(time(NULL) - timer, 1) << "The timer has not timed out at the right speed";
  }

  TEST_F (AsioTimerFixture, timerExampleAsync) {
    time_t timer;
    time(&timer);
    boost::asio::deadline_timer t(io, boost::posix_time::seconds(1));
    boost::asio::deadline_timer blocker(io, boost::posix_time::seconds(2));
    t.async_wait(boost::bind(asioTimeMarkCallback, boost::asio::placeholders::error, &mark));
    io.run();
    blocker.wait(); 
    ASSERT_EQ(mark - timer, 1) << "the async timer was flawed";
  }
} 
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
