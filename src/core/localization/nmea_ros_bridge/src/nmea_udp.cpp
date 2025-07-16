/*
 * Copyright 2021 Map IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// BSD 3-Clause License

// Copyright (c) 2021, MeijoMeguroLab
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/*
 * nmea_udp.cpp
 *
 * Author Kashimoto
 * Ver 1.00 2021/03/19
 */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"

#define BUFFER_SAFE 2000

#include "rclcpp/time.hpp"
#include <chrono>
#include "rclcpp/clock.hpp"

#include "rclcpp/parameter.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/logger.hpp"

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr pub;

struct sockaddr_in dstAddr;
static bool isConnected = false;

static void packet_receive_rate(int fd, std::string frame_id, double rate)
{
  nmea_msgs::msg::Sentence nmea_msg;
  nmea_msg.header.frame_id = frame_id.c_str();
  rclcpp::Rate loop_rate(rate);

  int rem;
  int ret;
  char buffer[2048];
  char* w_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while (rclcpp::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      rclcpp::shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        rclcpp::shutdown();
      }
    }

    *w_buffer = '\0';
    char* r_buffer = buffer;

    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    while (1)
    {
      char* sentence = strchr(r_buffer, '$');
      if (sentence == NULL)
      {
        break;
      }
      char* end_sentence = strchr(sentence, '\r');
      if (end_sentence == NULL)
      {
        break;
      }
      *end_sentence = '\0';

      nmea_msg.header.stamp = ros_clock.now();
      nmea_msg.sentence = sentence;
      pub->publish(nmea_msg);
      r_buffer = end_sentence + 1;
    }

    rem = w_buffer - r_buffer;
    if (rem > BUFFER_SAFE)
    {
      RCLCPP_WARN(node->get_logger(), "Buffer over.Init Buffer.");
      rem = 0;
    }
    memmove(buffer, r_buffer, rem);
    w_buffer = buffer + rem;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  close(fd);
}

static void packet_receive_no_rate(int fd, std::string frame_id)
{
  nmea_msgs::msg::Sentence nmea_msg;
  nmea_msg.header.frame_id = frame_id.c_str();


  int rem;
  int ret;
  char buffer[2048];
  char* w_buffer = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while (rclcpp::ok())
  {
    errno = 0;
    ret = read(fd, w_buffer, buffer_end - w_buffer - 1);
    if (ret > 0)
    {
      if (strnlen(w_buffer, ret) != ret)
      {
        w_buffer = buffer;
        continue;
      }
      w_buffer += ret;
    }
    else if (ret == 0)
    {
      rclcpp::shutdown();
      close(fd);
      return;
    }
    else
    {
      if (errno == EAGAIN)
      {
        continue;
      }
      else
      {
        rclcpp::shutdown();
      }
    }

    *w_buffer = '\0';
    char* r_buffer = buffer;

    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    while (1)
    {
      char* sentence = strchr(r_buffer, '$');
      if (sentence == NULL)
      {
        break;
      }
      char* end_sentence = strchr(sentence, '\r');
      if (end_sentence == NULL)
      {
        break;
      }
      *end_sentence = '\0';

      nmea_msg.header.stamp = ros_clock.now();
      nmea_msg.sentence = sentence;
      // std::cout << "nmea_msg.sentence " << nmea_msg.sentence <<std::endl;
      pub->publish(nmea_msg);
      r_buffer = end_sentence + 1;
    }

    rem = w_buffer - r_buffer;
    if (rem > BUFFER_SAFE)
    {
      RCLCPP_WARN(node->get_logger(), "Buffer over.Init Buffer.");
      rem = 0;
    }
    memmove(buffer, r_buffer, rem);
    w_buffer = buffer + rem;

    rclcpp::spin_some(node);

  }
  close(fd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("nmea_udp");

  int sock;
  int result,val;

  // Read parameters
  std::string address = "192.168.30.10";
  int port = 62002;
  std::string nmea_topic = "nmea_sentence";
  std::string frame_id = "sentence";
  double rate = 0.0;

  node->declare_parameter("address","192.168.30.10");
  node->declare_parameter("port", 62002);
  node->declare_parameter("nmea_topic", "nmea_sentence");
  node->declare_parameter("frame_id", "sentence");
  node->declare_parameter("rate", 0.0);

  address = node->get_parameter("address").as_string();
  port = node->get_parameter("port").as_int();
  nmea_topic = node->get_parameter("nmea_topic").as_string();
  frame_id = node->get_parameter("frame_id").as_string();
  rate = node->get_parameter("rate").as_double();

  pub = node->create_publisher<nmea_msgs::msg::Sentence>(nmea_topic, 10);

  RCLCPP_INFO(node->get_logger(), "IP: %s", address.c_str());
  RCLCPP_INFO(node->get_logger(), "PORT: %d", port);
  RCLCPP_INFO(node->get_logger(), "NMEA_TOPIC: %s", nmea_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "FRAME_ID: %s", frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "RATE: %f", rate);

  sock = socket(AF_INET, SOCK_DGRAM, 0);

  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_family = AF_INET;
  dstAddr.sin_addr.s_addr = INADDR_ANY;
  dstAddr.sin_port = htons(port);

  rclcpp::Rate loop_rate(1.0);
  while (rclcpp::ok())
  {
    //result = connect(sock, (struct sockaddr *) &dstAddr, sizeof(dstAddr));
    result = bind(sock, (struct sockaddr *)&dstAddr, sizeof(dstAddr));
    if( result < 0 )
    {
      /* non-connect */
      RCLCPP_INFO(node->get_logger(), "CONNECT TRY... ");
    }
    else
    {
      /* connect */
      isConnected = true;
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  if ((isConnected == true)&&(rclcpp::ok()))
  {
    if( rate != 0.0 )
    {
      /* non-block *//* rclcpp::rate() */
      val = 1;
      ioctl(sock, FIONBIO, &val);
      packet_receive_rate(sock, frame_id, rate);
    }
    else
    {
      /* block */
      packet_receive_no_rate(sock, frame_id);
    }

  }

  return 0;
}
