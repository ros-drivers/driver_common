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

#ifndef __SERVER_H__
#define __SERVER_H__

#include <boost/function.hpp>
#include <ros/node_handle.h>
#include <dynamic_reconfigure2/ParameterDescription.h>
#include <dynamic_reconfigure2/ParameterSet.h>
#include <dynamic_reconfigure2/Reconfigure.h>

/**
 * @todo Add diagnostics.
 */

namespace dynamic_reconfigure
{
/**
 * Keeps track of the reconfigure callback function.
 */
template <class ConfigType>  
class Server
{
public:
  Server(NodeHandle nh) :
    node_handle_(nh)
  {
    set_service_ = node_handle_.advertiseService("set_parameters",
        &Server<ConfigManipulator>::setConfigCallback, this);
    
    descr_pub_ = node_handle_.advertise<dynamic_reconfigure2::ConfigurationDescription>
      ("parameter_description", 1, true);
    dynamic_reconfigure2::ConfigurationDescription descr;
    ConfigType::__getDescriptionMessage__(descr);
    descr_pub_.publish(descr);
    
    update_pub_ = node_handle_.advertise("parameter_changes", 1, true);
    ConfigType init_config = ConfigType::__defaults__;
    init_config.__fromServer__();
    updateConfig(init_config);
  }

  typedef boost::function<void(ConfigType &, int level)> CallbackType;
  
  void setCallback(CallbackType &callback)
  {
    boost::mutex::scoped_lock lock(mutex_);
    callback_ = callback;
    if (callback) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
      callback(~0);
    else
      ROS_INFO("setCallback did not call callback because it was zero."); /// @todo kill this line.
  }

  void clearCallback()
  {
    boost::mutex::scoped_lock lock(mutex_);
    callback_.clear();
  }

  void updateConfig(const ConfigType &config)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    config_.__toServer__();
    update_pub_.publish(config_);
  }

private:
  ros::NodeHandle node_handle_;
  ros::ServiceServer set_service_;
  ros::Publisher update_pub_;
  ros::Publisher descr_pub_;
  CallbackType callback_;
  ConfigType config_;
  boost::mutex mutex_;

  bool setConfigCallback(typename ConfigManipulator::SetService::Request &req, 
      typename ConfigManipulator::SetService::Response &rsp)
  {
    boost::mutex::scoped_lock lock(mutex_);

    class ConfigManipulator::ConfigType new_config = req.config;
    ConfigManipulator::clamp(new_config);
    int level = config_.ConfigManipulator::getChangeLevel(config_);
    
    if (callback_)
      callback_(new_config, level);

    updateConfig(new_config)
    rsp.config = new_config;
    return true;
  }
};

template <class ConfigManipulator>
class Server : public GenericServerUnchecked
{
public:
  Server(const ros::NodeHandle &nh) : node_handle_(nh)
  {
    config_ = ConfigManipulator::getDefaults();
    ConfigManipulator::readFromParamServer(node_handle_, config_);
    ConfigManipulator::clamp(config_);
    // Write to make sure everything is filled in.
    ConfigManipulator::writeToParamServer(node_handle_, config_);
    
    static const std::string get_config = "get_configuration";
    get_service_ = node_handle_.advertiseService(get_config, &Server<ConfigManipulator>::getConfigService, this);
  }

  void getConfig(class ConfigManipulator::ConfigType &config)
  {
    config = config_;
  }

  void setConfig(const class ConfigManipulator::ConfigType &config)
  {
    config_ = config;
    ConfigManipulator::writeToParamServer(node_handle_, config_);
  }

private:
  bool getConfigService(typename ConfigManipulator::GetService::Request &req, 
      typename ConfigManipulator::GetService::Response &rsp)
  {
    rsp.config = config_;
    rsp.defaults = ConfigManipulator::getDefaults();
    rsp.min = ConfigManipulator::getMin();
    rsp.max = ConfigManipulator::getMax();
    return true;
  }


  class ConfigManipulator::ConfigType config_;
  ros::ServiceServer get_service_;
};
                                 
}
#endif
