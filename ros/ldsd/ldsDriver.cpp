/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

 /* Authors: SP Kong, JH Yang */
 /* maintainer: Pyo */

#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include "ldsDriver.hpp"

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  // Below command is not required after firmware upgrade (2017.10)
  boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
  boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
}

// This sensor is currently transmitting data every 6 degrees. 
// It doesn't transmit whole 360 degrees of data. 
// The improved code below transfers data at a faster rate than the previous one, 
// but it is updated with only 6 degrees data. The bandwidth also increases 60 times. (15KB/s > 900KB/s)
void LFCDLaser::poll(sensor_msgs::LaserScan& scan)
{
  uint8_t temp_char;
  bool got_scan = false;
  boost::array<uint8_t, 42> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  while (!shutting_down_ && !got_scan)
  {
    // Wait until first data sync of frame: 0xFA, 0xA0
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[0], 1));

    if(raw_bytes[0] == 0xFA)
    {
      // Now that entire start sequence has been found, read in the rest of the message
      got_scan = true;
      boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[1], 41));
      
      if(raw_bytes[1] >= 0xA0  && raw_bytes[1] <= 0xDB)
      {
        int degree_count_num = 0;
        index = (raw_bytes[1] - 0xA0) * 6;
        good_sets++;

        motor_speed += (raw_bytes[3] << 8) + raw_bytes[2]; //accumulate count for avg. time increment
        rpms=(raw_bytes[3]<<8|raw_bytes[2])/10;

        //read data in sets of 6
        for(uint16_t j = 4; j < 40; j = j + 6)
        {
          uint8_t byte0 = raw_bytes[j];
          uint8_t byte1 = raw_bytes[j+1];
          uint8_t byte2 = raw_bytes[j+2];
          uint8_t byte3 = raw_bytes[j+3];

          uint16_t intensity = (byte1 << 8) + byte0;
          uint16_t range     = (byte3 << 8) + byte2;

          uint16_t idx = ((269 - index - degree_count_num) + 359)%359; // idx=0 correspond to robot pos theta=0

          scan.ranges[idx] = range;
          scan.intensities[idx] = intensity;

          degree_count_num++;
        }      

        scan.time_increment = motor_speed/good_sets/1e8;
      }
    }
  }
}
}