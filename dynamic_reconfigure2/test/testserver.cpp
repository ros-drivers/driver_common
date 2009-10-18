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
  dynamic_reconfigure::Server srv;
  srv.setCallback(&callback);
  ros::spin();
  return 0;
}
