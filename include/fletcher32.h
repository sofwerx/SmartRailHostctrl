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
 *        License:  Creative Commons ShareAlike License
 *     Maintainer:  Gary Hendrick gary.hendrick.fellow@sofwerx.org
 *   Organization:  SOFWerx
 *         Origin:  http://en.wikipedia.org/wiki/Fletcher%27s_checksum#Test_vectors
 * =====================================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint32_t fletcher32( uint8_t *buf, uint32_t bufLen)
{
  uint32_t len   = bufLen % 2 ? bufLen+1 : bufLen;
  uint32_t sum1  = 0xffff, sum2 = 0xffff;
  uint16_t *data = (uint16_t *)buf;

  len = len>>1;

  while (len)
  {
    unsigned tlen = len > 360 ? 360 : len;
    len -= tlen;
    do
    {
      sum1 += *data++;
      sum2 += sum1;
    } while (--tlen);
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }
  /* Second reduction step to reduce sums to 16 bits */
  sum1 = (sum1 & 0xffff) + (sum1 >> 16);
  sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  return sum2 << 16 | sum1;
}
