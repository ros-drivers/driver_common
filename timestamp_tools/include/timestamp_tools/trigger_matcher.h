#ifndef __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
#define __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__

#include <ros/roslib.h>
#include <queue>
#include <pair>
#include <boost/thread.h>

namespace timestamp_tools
{

template <class C>
class TriggerMatcher
{
  typedef std::pair<ros::Time, boost::shared_ptr<C> > DataPair;
  typedef boost::function<void, ros::Time, boost::shared_ptr<C> > MatchCallback;

private:
  boost::mutex mutex_;

  std::queue<ros::Time> trig_queue_;
  std::queue<DataPair> data_queue_;
  
  ros::Duration trig_delay_;

  ros::Time last_data_stamp_;

  MatchCallback matchCallback_;
  int late_data_count_allowed_;
  int late_data_count_;
  bool locked_;

  bool nonCausalHeads()
  { // Are the heads non-causal? (Assumes the heads are present.)
    return trig_queue_.front() > data_queue_.front().first();
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
        data_queue_.pop_front();
        continue;
      }
      
      ros::Time trig_stamp;
      
      // If we are not locked, then we need to check that the next trigger
      // time stamp would not be satisfactory.
      if (!locked_)
      {
        while (true)
        {
          if (trig_queue.length() < 2)
            return; // Can't do the check now.

          trig_stamp = trig_queue_.pop_front();
          if (nonCausalHeads()) 
            break; // Head is non-causal, we have arrived.
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
        if (last_data_stamp_ > trig_queue_.head())
        {
          if (++late_data_count_ > late_data_count_allowed_)
          {
            locked_ = false;
            continue;
          }
        }
        else
          late_data_count_ = 0;

        trig_stamp = trig_queue_.pop_front();
      }
      
      // Call the callback
      C &data = data_queue_.pop_front().second();
      matchCallback_(trig_stamp, data.second());
      last_data_stamp_ = data.first();
    }
  }

public:
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

  TriggerMatcher(int late_data_count_allowed) : 
    trig_delay_(0), 
    late_data_count_allowed_(late_data_count_allowed),
    late_data_count_(0),
    last_data_stamp_(TIME_MIN),
    locked_(false)
  {
  }

  void triggerCallback(const roslib::HeaderPtr &msg)
  {
    triggerCallback(msg.stamp);
  }

  void triggerCallback(double stamp)
  {
    triggerCallback(ros::Time(stamp));
  }

  void triggerCallback(const ros::Time &stamp)
  {
    boost::mutex::scoped_lock(mutex_);
    
    trig_queue.push_back(stamp + trig_delay_);
    tryPop();
  }

  void dataCallback(double stamp, const boost::shared_ptr<C> &data)
  {
    dataCallback(std::pair(ros::Time(stamp), data));
  }

  void dataCallback(const ros::Time &stamp, const boost::shared_ptr<C> &data)
  {
    dataCallback(std::pair(stamp, data));
  }

  void DataCallback(const DataPair &pair)
  {
    boost::mutex::scoped_lock(mutex_);

    data_queue.push_back(pair);
    tryPop();
  }
};

}

#endif // __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
