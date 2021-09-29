#include <particle_filter_localization/particle_filter_localization.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter_localization_node");
  ParticleFilterLocalization particle_filter_localization;
  ros::spin();
  return 0;
}
