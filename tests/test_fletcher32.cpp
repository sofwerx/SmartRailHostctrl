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

#include <gtest/gtest.h>
#include "fletcher32.h"

namespace {
  TEST(Fletcher32, abcde)
  {
    char str[] = "abcde";
    size_t len = strlen(str);

    uint8_t as_ints[len];

    for(int i=0; i <= len; i++) {
      as_ints[i] = str[i];
    };

    ASSERT_EQ(0xF04FC729, fletcher32(as_ints, len));
  }
  
  TEST(Fletcher32, abcdef)
  {
    char str[] = "abcdef";
    size_t len = strlen(str);

    uint8_t as_ints[len];

    for(int i=0; i <= len; i++) {
      as_ints[i] = str[i];
    };

    ASSERT_EQ(0x56502D2A, fletcher32(as_ints, len));
  }

  TEST(Fletcher32, abcdefgh)
  {
    char str[] = "abcdefgh";
    size_t len = strlen(str);

    uint8_t as_ints[len];

    for(int i=0; i <= len; i++) {
      as_ints[i] = str[i];
    };

    ASSERT_EQ(0xEBE19591, fletcher32(as_ints, len));
  }
} // namespace 
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  entry point testing the fletcher32 calculator 
 * =====================================================================================
 */
int main ( int argc, char *argv[] )
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}				/* ----------  end of function main  ---------- */
