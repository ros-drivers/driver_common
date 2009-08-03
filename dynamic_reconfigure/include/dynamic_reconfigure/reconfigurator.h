/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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


/**

 Author: Blaise Gassend

 Handles synchronizing node state with the configuration server, and 
 handling of services to get and set configuration.

*/

#ifndef __RECONFIGURATOR_H__
#define __RECONFIGURATOR_H__

#include <boost/function.hpp>
#include <ros/node_handle.h>

namespace dynamic_reconfigure
{
/**
 * Keeps track of the reconfigure callback function.
 */
class AbstractReconfigurator
{
public:
  AbstractReconfigurator()
  {
  }
  
  void set_callback(const boost::function<void(int level)> &callback)
  {
    callback_ = callback;
    if (callback) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
      callback(~0);
  }

  void clear_callback()
  {
    callback_.clear();
  }

protected:
  boost::function<void(int level)> callback_;
};

template <class ConfigManipulator>
class Reconfigurator : public AbstractReconfigurator
{
public:
  Reconfigurator(ros::NodeHandle &nh) : node_handle_(nh)
  {
    config_ = ConfigManipulator::get_defaults();
    ConfigManipulator::read_from_param_server(node_handle_, config_);
    // Write to make sure everything is filled in.
    ConfigManipulator::write_to_param_server(node_handle_, config_);
    
    static const std::string get_config = "~get_configuration";
    get_service_ = node_handle_.advertiseService(get_config, &Reconfigurator<ConfigManipulator>::get_config_service, this);
    static const std::string set_config = "~set_configuration";
    set_service_ = node_handle_.advertiseService(set_config, &Reconfigurator<ConfigManipulator>::set_config_service, this);
  }

  void get_config(class ConfigManipulator::ConfigType &config)
  {
    config = config_;
  }

  void set_config(const class ConfigManipulator::ConfigType &config)
  {
    config_ = config;
    ConfigManipulator::write_to_param_server(node_handle_, config_);
  }

private:
  bool get_config_service(typename ConfigManipulator::GetService::Request &req, 
      typename ConfigManipulator::GetService::Response &rsp)
  {
    rsp.defaults = ConfigManipulator::get_defaults();
    rsp.min = ConfigManipulator::get_min();
    rsp.max = ConfigManipulator::get_max();
    return true;
  }

  bool set_config_service(typename ConfigManipulator::SetService::Request &req, 
      typename ConfigManipulator::SetService::Response &rsp)
  {
    int level = ConfigManipulator::get_change_level(req.config, config_);

    set_config(req.config);

    // We expect config_ to be read, and possibly written during the
    // callback.
    if (callback_)
      callback_(level);
    
    rsp.config = config_;

    return true;
  }

  class ConfigManipulator::ConfigType config_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer get_service_;
  ros::ServiceServer set_service_;
};
                                 
}
#endif
