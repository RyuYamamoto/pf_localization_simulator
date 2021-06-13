#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <particle_filter_localization/multi_variate_generator.h>

struct Particle
{
  Eigen::VectorXd vec;
};

class ParticleFilter
{
public:
  ParticleFilter(const int particle_num) : particle_num_(particle_num)
  {
    particle_.resize(particle_num);
  }
  ~ParticleFilter() = default;

  std::size_t getParticleSize() { return particle_.size(); }
  Particle getParticle(std::size_t idx) { return particle_.at(idx); }

  void setBasePose(const Eigen::VectorXd vec)
  {
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      particle_.at(idx).vec = vec;
    }
  }

  void setParticleNum(const int particle_num) { particle_num_ = particle_num; }

  void update(
    const double velocity, const double omega, const double dt, const Eigen::Vector4d motion_noise)
  {
    MultiVariateNormal multi_variate_normal(Eigen::VectorXd::Zero(motion_noise.rows()), motion_noise.asDiagonal());
    for (std::size_t idx = 0; idx < getParticleSize(); ++idx) {
      Eigen::VectorXd noise = multi_variate_normal();
      const double noise_vel = velocity + noise(0) * std::sqrt(std::fabs(velocity)/dt) + noise(1) * std::sqrt(std::fabs(omega)/dt);
      const double noise_omega = omega + noise(2) * std::sqrt(std::fabs(velocity)/dt) + noise(3) * std::sqrt(std::fabs(omega)/dt);

      particle_.at(idx).vec(2) += noise_omega * dt;
      particle_.at(idx).vec(0) += noise_vel * std::cos(particle_.at(idx).vec(2)) * dt;
      particle_.at(idx).vec(1) += noise_vel * std::sin(particle_.at(idx).vec(2)) * dt;
    }
  }

private:
  int particle_num_;

  std::vector<Particle> particle_;
};

#endif
