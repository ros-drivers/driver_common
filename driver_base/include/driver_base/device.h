/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

// Author: Blaise Gassend
#ifndef __DRIVER_BASE__DEVICE_H__
#define __DRIVER_BASE__DEVICE_H__

namespace driver_base
{

/**
 *
 * State transition functions assume that they are called from the correct
 * state. It is the caller's responsibility to check that the state is
 * correct. The Device mutex_ should always be locked by the caller before
 * checking the current state and throughout the call to the transition function.
 *
 * State should not change between calls to the transition function except
 * for transitions from RUNNING to STOPPED. The device must hold its lock
 * while doing this transition to allow the caller to enforce that stop is
 * only called from the RUNNING state.
 *
 * Transitions may fail. It is the caller's responsibility to check success
 * by looking at the device's state_. After a failure, the device can set
 * itself into any state. For example, if a call to start() fails, the
 * device may end up in the CLOSED state.
 *
 */

class Device
{
protected:
  char state_;

public:
  boost::mutex mutex_;
  
  virtual void open() = 0;
  virtual void close() = 0;
  
  virtual void start() = 0;
  virtual void stop() = 0;
  
  typedef char state_t;

  static const state_t CLOSED = 0; // Not connected to the hardware.
  static const state_t OPENED = 1; // Connected to the hardware, ready to start streaming.
  static const state_t RUNNING = 2; // Streaming data.

  state_t getState()
  {
    return state_;
  }

  const std::string getStateName()
  {
    return getStateName(state_);
  }
  
  static const std::string getStateName(state_t s)
  {
    std::string name[4] = {
      std::string("CLOSED"),
      std::string("OPENED"),
      std::string("RUNNING"),
      std::string("Unknown")
    };

    if (s >= 0 && s <= 2)
      return name[(int) s];
    else
      return name[3];
  }

  virtual std::string getID() = 0;

  virtual ~Device() {}
};

/// @fixme derived classes for nodes that only poll or only stream.

};

#endif

