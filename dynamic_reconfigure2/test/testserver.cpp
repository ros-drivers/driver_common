#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure2/TestConfig.h>

void callback(dynamic_reconfigure2::TestConfig &config, uint32_t level)
{
  config.int_ |= 1;
  config.double_ = -config.double_;
  config.str_ += "A";
  config.bool_ = !config.bool_;
  config.level = level;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_test_server");
  dynamic_reconfigure::Server<dynamic_reconfigure2::TestConfig> srv;
  dynamic_reconfigure::Server<dynamic_reconfigure2::TestConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
