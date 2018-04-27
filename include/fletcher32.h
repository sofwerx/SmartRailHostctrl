/*
 * =====================================================================================
 *
 *       Filename:  fletcher32.h
 *
 *    Description:  a fletcher 32 calculator
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
 *  SOFTWARE.*
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint32_t fletcher32( uint8_t const *data, uint32_t len )
{
  uint32_t sum1 = 0, sum2 = 0;
  int index;
  for (index = 0; index <len; ++index )
  {
    sum1 = (sum1 + data[index]) % 0xffffffff;
    sum2 = (sum2 + sum1) % 0xffffffff;
  }
  return (sum2 << 16) | sum1;
}
/*
   uint32_t
   fletcher32(const uint16_t *data, size_t len)
   {
   uint32_t c0, c1;
   unsigned int i;

   for (c0 = c1 = 0; len >= 360; len -= 360) {
   for (i = 0; i < 360; ++i) {
   c0 = c0 + *data++;
   c1 = c1 + c0;
   }
   c0 = c0 % 65535;
   c1 = c1 % 65535;
   }
   for (i = 0; i < len; ++i) {
   c0 = c0 + *data++;
   c1 = c1 + c0;
   }
   c0 = c0 % 65535;
   c1 = c1 % 65535;
   return (c1 << 16 | c0);
   } */
