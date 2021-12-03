#include <particle_filter_localization/particle_filter_localization.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParticleFilterLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
