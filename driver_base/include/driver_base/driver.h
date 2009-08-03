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
#ifndef __DRIVER_BASE__DRIVER_H__
#define __DRIVER_BASE__DRIVER_H__

#include <diagnostic_updater/diagnostic_updater.h>
#include <self_test/self_test.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace driver_base
{

class AbstractDriver
{
  static int ctrl_c_hit_count_; 
  
  static void sigCalled(int sig)
  {
    ctrl_c_hit_count_++;
  }
};
  
template <class DriverType>
int main(int argc, char **argv, std::string name)
{
  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  signal(SIGINT, &AbstractDriver::sigCalled);
  signal(SIGTERM, &AbstractDriver::sigCalled);
  ros::NodeHandle nh;
  DriverType driver(nh);
  return driver.spin();
  /// @todo Add check for leaked file descriptors and such things here.
}
  
template <class Device>
class Driver : public AbstractDriver
{
public:
  typedef char state_t;
 
protected:
  // Hooks
  virtual void add_diagnostics() {}
  virtual void add_stopped_tests() {}
  virtual void add_opened_tests() {}
  virtual void add_running_tests() {};

  // Helper classes
  ros::NodeHandle node_handle_;
  self_test::Dispatcher<Driver<Device> > self_test_;
  diagnostic_updater::Updater diagnostic_;
  typename Device::Reconfigurator reconfigurator_;
  
  Device device_;

private:
  // Subscriber tracking
  int num_subscribed_topics_; // Number of topics that have subscribers.
  
  static const state_t DISABLED = 0;
  static const state_t LAZY_ON = 1;
  static const state_t ALWAYS_ON = 2;
  static const state_t SELF_TEST = 3;
  static const state_t EXITING = 4;

  state_t state_;
                          
  boost::shared_ptr<boost::thread> ros_thread_;
  
  int exit_status_;

  // The device
  typedef typename Device::state_t dev_state_t;

  dev_state_t pre_self_test_device_state_;

  void reconfigure(int level)
  {
    boost::mutex::scoped_lock(device_.mutex_);
    
    dev_state_t orig_state = device_.getState();
  
    if ((level | dynamic_reconfigure::SensorLevels::RECONFIGURE_STOP) == level)
    {
      stop();
      if (!is_stopped())
        ROS_ERROR("Failed to stop streaming before reconfiguring. Reconfiguration may fail.");
    }
  
    if ((level | dynamic_reconfigure::SensorLevels::RECONFIGURE_CLOSE) == level)
    {
      close();
      if (!is_closed())
        ROS_ERROR("Failed to close device before reconfiguring. Reconfiguration may fail.");
    }
  
    reconfigurator_.get_config(device_.config_);
    device_.config_update();
    reconfigure_hook(level);
    reconfigurator_.set_config(device_.config_);
  
    go_state(orig_state);

    if (device_.getState() != orig_state)
    {
      ROS_ERROR("Failed to resume original device state after reconfiguring. The requested configuration may contain errors.");
    }
  }

  virtual void reconfigure_hook(int level) {}

  /*

  void diagnosticsLoop()
  {
    //int frameless_updates = 0;

    bool have_started = false;

    cam_pub_.clear_window(); // Avoids having an error until the window fills up.
    while (node_handle_.ok())
    {
      if (!started_video_)
      {
        stop();
        close();
        if (have_started && config_.exit_on_fault)
        {
          node_handle_.shutdown();
          break;
        }
        open();
        start();
        have_started = true;
      }

      {
        boost::mutex::scoped_lock(diagnostics_lock_);
        diagnostic_.update();
        self_test_.checkTest();
      }
      sleep(1);
    }

    ROS_DEBUG("Diagnostic thread exiting.");
  }

   */

protected: 
  bool go_state(dev_state_t target)
  {
    boost::mutex::scoped_lock(device_.mutex_);

    dev_state_t cur_state = device_.getState();
    
    if (cur_state > target)
      return lower_state(target);

    if (cur_state < target)
      return raise_state(target);

    return true;
  }

  bool try_transition(dev_state_t target, void (Device::*transition)())
  {
    boost::mutex::scoped_lock(device_.mutex_);
    (device_.*transition)();
    return device_.getState() == target;
  }

  bool raise_state(dev_state_t target)
  {
    boost::mutex::scoped_lock(device_.mutex_);

    switch (device_.getState())  
    {
      case Device::CLOSED:
        if (target <= Device::CLOSED)
          return true;
        if (!try_transition(Device::OPENED, &Device::open))
          return false;

      case Device::OPENED:
        if (target <= Device::OPENED)
          return true;
        if (!try_transition(Device::RUNNING, &Device::start))
          return false;

      default:
        return target <= device_.getState();
    }
  }

  bool lower_state(dev_state_t target)
  {
    boost::mutex::scoped_lock(device_.mutex_);

    switch (device_.getState())  
    {
      case Device::RUNNING:
        if (target >= Device::RUNNING)
          return true;
        if (!try_transition(Device::OPENED, &Device::stop))
          return false;

      case Device::OPENED:
        if (target >= Device::OPENED)
          return true;
        if (!try_transition(Device::CLOSED, &Device::close))
          return false;

      default:
        return target >= device_.getState();
    }
  }

  void go_running()
  {
    go_state(Device::RUNNING);
  }

  void go_opened()
  {
    go_state(Device::OPENED);
  }

  void go_closed()
  {
    go_state(Device::CLOSED);
  }

  void stop()
  {
    lower_state(Device::OPENED);
  }

  void start()
  {
    raise_state(Device::RUNNING);
  }
  
  void open()
  {
    raise_state(Device::OPENED);
  }
  
  void close()
  {
    lower_state(Device::CLOSED);
  }
  
  bool is_running()
  {
    return device_.getState() == Device::RUNNING;
  }

  bool is_opened()
  {
    return device_.getState() == Device::OPENED;
  }

  bool is_closed()
  {
    return device_.getState() == Device::CLOSED;
  }
  
  bool is_stopped()
  {
    dev_state_t s = device_.getState();
    return s == Device::CLOSED || s == Device::OPENED;
  }

private:
/*  void connectCallback(const ros::PublisherPtr &pub)
  {
    if (pub.numSubscribers == 1)
      num_subscribed_topics_++;

    if (num_subscribed_topics_ == 1)
      start();
  }

  void disconnectCallback(const ros::PublisherPtr &pub)
  {
    if (pub.numSubscribers == 0)
      num_subscribed_topics_++;

    if (num_subscribed_topics_ == 0)
      stop();
  }*/

  void prepare_diagnostics()
  {
    diagnostic_.add(ros::this_node::getName() + ": Driver Status", this, &Driver::status_diagnostic);
    add_diagnostics();
  }

  void status_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(1, "Driver is okay.");
    
    stat.adds("Device state:", device_.getStateName());
    /// @fixme need to put something more useful here.
  }

  void prepare_self_tests()
  {
    self_test_.add( "Interruption Test", &Driver::interruptionTest );
    add_stopped_tests();
    self_test_.add( "Connection Test", &Driver::openTest );
    add_opened_tests();
    self_test_.add( "Start Streaming Test", &Driver::runTest );
    add_running_tests();
    self_test_.add( "Stop Streaming Test", &Driver::stopTest );
    self_test_.add( "Disconnection Test", &Driver::closeTest );
    self_test_.add( "Resume Activity", &Driver::resumeTest );
  } 

  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    pre_self_test_device_state_ = device_.getState();
    go_closed();
    
    if (num_subscribed_topics_ > 0)
      status.summary(1, "There were active subscribers.  Running of self test interrupted operations.");
    else if (is_closed())
      status.summary(2, "Could not close device.");
    else
      status.summary(0, "No operation interrupted.");
  } 
    
  void openTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    open();

    if (is_opened())
      status.summaryf(0, "Successfully opened device %s", device_.getID().c_str());
    else
      status.summary(2, "Failed to open.");
  }

  void runTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    start();
    
    if (is_running())      
      status.summaryf(0, "Successfully started streaming.");
    else
      status.summary(2, "Failed to start streaming.");
  } 

  void stopTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    stop();
    
    if (is_opened())      
      status.summaryf(0, "Successfully stopped streaming.");
    else
      status.summary(2, "Failed to stop streaming.");
  } 
  
  void closeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    close();
    
    if (is_closed())      
      status.summaryf(0, "Successfully closed device.");
    else
      status.summary(2, "Failed to close device.");
  } 
  
  void resumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    go_state(pre_self_test_device_state_);
  
    dev_state_t currentState = device_.getState();
    std::string desired_state_name = Device::getStateName(pre_self_test_device_state_);
    std::string current_state_name = Device::getStateName(currentState);
   
    if (currentState == pre_self_test_device_state_)
      status.summaryf(0, "Successfully returned to %s state.", desired_state_name.c_str());
    else
      status.summaryf(2, "Failed to return to %s state, now in state %s.", desired_state_name.c_str(), current_state_name.c_str());
  }
  
public:
  
  virtual ~Driver() {}

  int spin()
  {
    ros_thread_.reset(new boost::thread(boost::bind(&ros::spin)));
    /// @todo What happens if thread creation fails?
    assert(ros_thread_);
                     
    /// @todo Do something about exit status?
    while (node_handle_.ok() && state_ != EXITING && !ctrl_c_hit_count_)
    {
      go_running();
      /// Will need some locking here or in diagnostic_updater?
      diagnostic_.update();
      self_test_.checkTest();
      sleep(1); /// @todo use a ROS sleep here?
    }
  
    go_closed();
    
    if (ros_thread_ && !ros_thread_->timed_join((boost::posix_time::milliseconds) 2000))
    {
      ROS_ERROR("ROS thread did not die after two seconds. Pretending that it did. This is probably a bad sign.");
    }
    ros_thread_.reset();

    return 0; /// @todo Work on return type here.
  }
  
  Driver(ros::NodeHandle &nh) : node_handle_(nh), self_test_(this, node_handle_), diagnostic_(node_handle_), reconfigurator_(node_handle_)
  {
    num_subscribed_topics_ = 0; /// @fixme this variable is hoakey.
    exit_status_ = 0;
    prepare_diagnostics();
    prepare_self_tests();
    reconfigurator_.set_callback(boost::bind(&Driver::reconfigure, this, _1));
  }
};

int AbstractDriver::ctrl_c_hit_count_ = 0;
  
// @todo exit status.
// @todo take over ctrl_c.

};

#endif
