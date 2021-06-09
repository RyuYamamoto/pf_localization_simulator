#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

struct Particle
{
  geometry_msgs::Pose pose;
};

class ParticleFilter
{
public:
  ParticleFilter(const int particle_num) : particle_num_(particle_num) { particle_.resize(particle_num); }
  ~ParticleFilter() = default;

  std::size_t getParticleSize() { return particle_.size(); }
  Particle getParticle(std::size_t idx) { return particle_.at(idx); }

  void setBasePose(const geometry_msgs::Pose pose)
  {
    for(std::size_t idx=0;idx<getParticleSize();++idx) {
      particle_.at(idx).pose = pose;
    }
  }

private:
  int particle_num_;

  std::vector<Particle> particle_;
};
