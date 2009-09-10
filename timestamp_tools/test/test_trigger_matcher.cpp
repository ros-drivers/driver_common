#include <timestamp_tools/trigger_matcher.h>
#include <gtest/gtest.h>
#include <queue>

class Checker
{
  std::queue< std::pair<double, int> > incoming_;

public:  
  timestamp_tools::TriggerMatcher<int> tm;
  
  Checker() : tm(1, 10, 5)
  {
    tm.setTrigDelay(0.2);
    timestamp_tools::TriggerMatcher<int>::MatchCallback bound_cb = boost::bind(&Checker::callback, this, _1, _2);
    tm.setMatchCallback(bound_cb);
                  
    // Get it into the permanent regime with an empty queue.
    tm.triggerCallback(0);
    expectEmpty(0);
    tm.dataCallback(1, 1);
    expectEmpty(1);
    tm.triggerCallback(2);
    expectHead(0.2, 1);
    tm.dataCallback(3, 2);
    expectHead(2.2, 2);

  }

  void expectEmpty(double time)
  {
    EXPECT_TRUE(incoming_.empty()) << "Checker queue not empty at time " << time 
      << " contains " << incoming_.front().first << ", " << incoming_.front().second;
  }

  void expectHead(double time, int data)
  {
    if (incoming_.empty())
    {
      ADD_FAILURE() << "Checker queue empty when checking " << time << ", " << data;
      return;
    }

    std::pair<double, int> &head = incoming_.front();

    EXPECT_EQ(time, head.first) << "Timestamp mismatch when checking " << time << ", " << data;
    EXPECT_EQ(data, head.second) << "Data mismatch when checking " << time << ", " << data;

    incoming_.pop();
  }

  void callback(const ros::Time &time, const boost::shared_ptr<int> &data)
  {
    incoming_.push(std::pair<double, int>(time.toSec(), *data));
  }
};

TEST(TriggerMatcher, BasicFunctionality)
{
  Checker c;
  
// Data gets delayed...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.dataCallback(5, 3);
  c.expectHead(4.2, 3);
  c.tm.dataCallback(7, 4);
  c.expectHead(6.2, 4);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);

// Timestamp gets delayed...
  c.tm.dataCallback(11, 6);
  c.expectEmpty(11);
  c.tm.dataCallback(13, 7);
  c.expectEmpty(13);
  c.tm.dataCallback(15, 8);
  c.expectEmpty(15);
  c.tm.triggerCallback(10);
  c.expectHead(10.2, 6);
  c.tm.triggerCallback(12);
  c.expectHead(12.2, 7);
  c.tm.triggerCallback(14);
  c.expectHead(14.2, 8);

  c.expectEmpty(1000);
}

TEST(TriggerMatcher, MissingTrigger)
{
  Checker c;
  
  // Miss a trigger at time 4...
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.dataCallback(5, 3);
  c.expectEmpty(5);
  c.tm.dataCallback(7, 4);
  c.expectHead(6.2, 4);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);
  
  c.expectEmpty(1000);
}

TEST(TriggerMatcher, MissingData)
{
  Checker c;
  
  // Miss data at time 5...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.triggerCallback(10);
  c.expectEmpty(10);
  c.tm.triggerCallback(12);
  c.expectEmpty(12);
  c.tm.dataCallback(7, 4);
  c.expectHead(4.2, 4); // Bad
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5); // Recovered
  c.tm.dataCallback(11, 6);
  c.expectHead(10.2, 6); 
  c.tm.dataCallback(13, 7);
  c.expectHead(12.2, 7); 
  
  c.expectEmpty(1000);
}

TEST(TriggerMatcher, MissingDataZeroLateTolerance)
{
  Checker c;
  
  c.tm.setLateDataCountAllowed(0);

  // Miss data at time 5...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.triggerCallback(10);
  c.expectEmpty(10);
  c.tm.triggerCallback(12);
  c.expectEmpty(12);
  c.tm.triggerCallback(14);
  c.expectEmpty(14);
  c.tm.dataCallback(5, 4);
  c.expectHead(4.2, 4); 
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5); // Recovered
  c.tm.dataCallback(11, 6);
  c.expectHead(10.2, 6); 
  c.tm.dataCallback(13, 7);
  c.expectHead(12.2, 7); 
  
  c.expectEmpty(1000);
}

TEST(TriggerMatcher, TriggerQueueOverflow)
{
  Checker c;

  double trig_time = 4;
  for (int i = 0; i < 15; i++)
  {
    c.tm.triggerCallback(trig_time);
    c.expectEmpty(trig_time);
    trig_time += 2;
  }
  c.tm.dataCallback(15, 15);
  c.expectHead(14.2, 15); // Previous triggers were dropped

  c.expectEmpty(1000);
}

TEST(TriggerMatcher, DataQueueOverflow)
{
  Checker c;

  double data_time = 5;
  for (int i = 0; i < 10; i++)
  {
    c.tm.dataCallback(data_time, data_time);
    c.expectEmpty(data_time);
    data_time += 2;
  }
  c.tm.triggerCallback(14);
  c.expectHead(14.2, 15); // Previous triggers were dropped

  c.expectEmpty(1000);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
