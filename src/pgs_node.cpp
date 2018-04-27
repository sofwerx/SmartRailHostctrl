/*
 * =====================================================================================
 *
 *       Filename:  pgs_node.cpp
 *
 *    Description:  pgs_node implements a serial listener for the PGS node
 *    along with a publisher to push that data into the appropriate
 *    communications infrastructure
 *
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
 *  SOFTWARE.
 */

#include "smartrail_hostctrl/serial_protocol.h"
#include <boost/asio.hpp>
#include <ros/console.h>
#include <ros/ros.h>

using ros::param::param;
using boost::asio::io_service;
using std::string;
using smartrail_hostctrl::SerialPgsSession;
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  an entry point into the ros node, note that the ros::param configuration
 will permit the use of a launch file to modify the binding behavior
 of the port. 
 * =====================================================================================
 */
int main ( int argc, char *argv[] )
{
  ros::init(argc, argv, "smartrail_hostctrl_pgs_node");
  ros::NodeHandle nh;

  string port="";
  int baud=9600;
  int character_size=8;
  bool flow_control=false;
  bool parity=false;
  int stop_bits=1; 

  if (!nh.hasParam("/pgs_node/port")) {
    ROS_ERROR("pgs_node has not been supplied a port value, exiting without.");
    return 0;
  }

  // bring in the parameters from the param server  
  nh.getParam("/pgs_node/port", port);
  nh.getParam("/pgs_node/baud", baud);
  nh.getParam("pgs_node/csize", character_size);
  nh.getParam("/pgs_node/flow", flow_control);
  nh.getParam("/pgs_node/parity", parity);
  nh.getParam("/pgs_node/stop_bits", stop_bits);

  // a little debug for the configuration
  ROS_DEBUG("Parameters set and ready to establish connection using (port=%s, baud=%d, csize=%d,\
    flow=%d, parity=%d, stop_bits=%d", port.c_str(), baud, character_size, flow_control, parity, stop_bits);
  // initialize an io_service and a serial session before entering into the run call
  io_service io;
  SerialPgsSession serial_session(io, port, baud, character_size, flow_control, parity, stop_bits);
  io.run();
  return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
