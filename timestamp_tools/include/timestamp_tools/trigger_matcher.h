#ifndef __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
#define __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__

#include <ros/ros.h>
#include <queue>
#include <utility>
#include <boost/thread.hpp>
#include <roslib/Header.h>

namespace timestamp_tools
{

template <class C>
class TriggerMatcher
{
public:
  typedef std::pair<ros::Time, boost::shared_ptr<C> > DataPair;
  typedef boost::function<void(ros::Time, boost::shared_ptr<C>)> MatchCallback;

private:
  boost::mutex mutex_;

  std::queue<ros::Time> trig_queue_;
  std::queue<DataPair> data_queue_;
  
  ros::Duration trig_delay_;

  ros::Time last_data_stamp_;

  MatchCallback matchCallback_;
  unsigned int late_data_count_allowed_;
  unsigned int late_data_count_;
  unsigned int max_data_queue_length_;
  unsigned int max_trig_queue_length_;
  bool locked_;

  bool nonCausalHeads()
  { // Are the heads non-causal? (Assumes the heads are present.)
    return trig_queue_.front() > data_queue_.front().first;
  }
  
  void tryPop()
  {
    while (!trig_queue_.empty() && !data_queue_.empty())
    {
      // If the head trigger is after the head data then we don't have a
      // timestamp for the data and drop it.
      if (nonCausalHeads())
      {
        ROS_WARN("TriggerMatcher: data arrived before trigger. Discarding data sample.");
        data_queue_.pop();
        continue;
      }
      
      ros::Time trig_stamp = trig_queue_.front();
      
      // If we are not locked, then we need to check that the next trigger
      // time stamp would not be satisfactory. (If no late counts are
      // allowed then we are always unlocked.)
      if (!locked_ || !late_data_count_allowed_)
      {
        while (true)
        {
          if (trig_queue_.size() < 2)
            return; // Can't do the check now.

          trig_queue_.pop();
          if (nonCausalHeads()) 
            break; // Head is non-causal, we have arrived.
          trig_stamp = trig_queue_.front(); // Skip that trigger event.
        }
        locked_ = true;
        late_data_count_ = 0;
      }
      else 
      {
        // If the data is later than the next trigger timestamp, we might
        // have missed a timestamp, or the data may just have been delayed a
        // lot. We count it and if it happens too often we assume that a
        // timestamp was missed and redo the locking process.
        if (last_data_stamp_ > trig_stamp)
        {
          if (++late_data_count_ >= late_data_count_allowed_)
          {
            ROS_WARN("TriggerMatcher: too many late data packets. Assuming missed trigger. Relocking...");
            locked_ = false;
            continue;
          }
        }
        else
          late_data_count_ = 0;

        trig_queue_.pop();
      }
      
      // Call the callback
      DataPair &data = data_queue_.front();
      matchCallback_(trig_stamp, data.second);
      last_data_stamp_ = data.first;
      data_queue_.pop();
    }
  }

public:
  void setLateDataCountAllowed(unsigned int v)
  {
    late_data_count_allowed_ = v;
  }

  void setTrigDelay(double delay)
  {
    setTrigDelay(ros::Duration(delay));
  }

  void setTrigDelay(const ros::Duration &delay)
  {
    trig_delay_ = delay;
  }

  void setMatchCallback(MatchCallback &cb)
  {
    matchCallback_ = cb;
  }

  TriggerMatcher(unsigned int late_data_count_allowed, unsigned int max_trig_queue_length, unsigned int max_data_queue_length) : 
    trig_delay_(0), 
    last_data_stamp_(ros::TIME_MIN),
    late_data_count_allowed_(late_data_count_allowed),
    late_data_count_(0),
    max_data_queue_length_(max_data_queue_length),
    max_trig_queue_length_(max_trig_queue_length),
    locked_(false)
  {
  }

  void triggerCallback(const roslib::HeaderPtr &msg)
  {
    triggerCallback(msg->stamp);
  }

  void triggerCallback(double stamp)
  {
    triggerCallback(ros::Time(stamp));
  }

  void triggerCallback(const ros::Time &stamp)
  {
    boost::mutex::scoped_lock(mutex_);
    
    trig_queue_.push(stamp + trig_delay_);
    if (trig_queue_.size() > max_trig_queue_length_)
    {
      ROS_WARN("TriggerMatcher: trig_queue_ overflow dropping from front.");
      trig_queue_.pop();
    }
    tryPop();
  }

  void dataCallback(double stamp, const C &data)
  {
    dataCallback(ros::Time(stamp), data);
  }

  void dataCallback(const ros::Time &stamp, const C &data)
  {
    boost::shared_ptr<C> ptr = boost::shared_ptr<C>(new C(data));
    dataCallback(stamp, ptr);
  }

  void dataCallback(double stamp, const boost::shared_ptr<C> &data)
  {
    dataCallback(ros::Time(stamp), data);
  }

  void dataCallback(const ros::Time &stamp, const boost::shared_ptr<C> &data)
  {
    dataCallback(DataPair(stamp, data));
  }

  void dataCallback(const DataPair &pair)
  {
    boost::mutex::scoped_lock(mutex_);

    data_queue_.push(pair);
    if (data_queue_.size() > max_data_queue_length_)
    {
      ROS_WARN("TriggerMatcher: trig_queue_ overflow dropping from front.");
      data_queue_.pop();
    }
    tryPop();
  }
};

}

#endif // __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
