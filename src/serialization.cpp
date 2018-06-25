/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Steven Macenski
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Steven Macenski, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 * Purpose: Allow for weighted regions for traversal to be easily used
 *********************************************************************/

#ifndef SERIALIZATION_H_
#define SERIALIZATION_H_

#include <ros/ros.h>
#include <weighted_region_layer/data_serial.h>

namespace serialization
{

namespace ser = ros::serialization;

void Read(const std::string& filename, weighted_region_layer::data_serial& msg)
{
  std::ofstream ofs(filename, std::ios::out|std::ios::binary);

  uint32_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ser::OStream stream(buffer.get(), serial_size);
  ser::serialize(stream, msg);

  ofs.write((char*) buffer.get(), serial_size);
  ofs.close();
  return;
}

void Write(const std::string& filename, weighted_region_layer::data_serial& msg)
{
  std::ifstream ifs(filename, std::ios::in|std::ios::binary);
  ifs.seekg (0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg (0, std::ios::beg);
  std::streampos begin = ifs.tellg();
  uint32_t serial_size = end-begin;

  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]); 
  ifs.read((char*) buffer.get(), serial_size);

  ros::serialization::IStream istream(buffer.get(), serial_size);
  ros::serialization::deserialize(istream, msg);
  ifs.close();
  return;
}

} // end namespace

#endif
