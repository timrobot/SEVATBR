// using particle filters
#include <math.h>
#include "robot.h"

void set_noise(slambot_t *robot, double sensor_noise, double turn_noise, double motion_noise) {
  robot->sensornoise = sensor_noise;
  robot->turnnoise = turn_noise;
  robot->motionnoise = motion_noise;
}

// pretend there exists no noise
void move(slambot_t *robot, double theta, double radius) {
  robot->orientation.yaw += theta;
  robot->orientation.x += cos(r->yaw) * radius;
  robot->orientation.y += sin(r->yaw) * radius;
}

// sense the environment for landmarks
void sense(slambot_t *robot) {
  particle_resample(robot);
}

void particle_update(slambot_t *robot, pose3d_t *trajectory) {
  int i;
  for (i = 0; i < robot->numparticles; i++) {
    robot->particles[i].x += trajectory.x +
      gauss_error(0.0, robot->motionnoise);
    robot->particles[i].y += trajectory.y +
      gauss_error(0.0, robot->motionnoise);
    robot->particles[i].yaw += trajectory.yaw +
      gauss_error(0.0, robot->turnnoise);
  }
  robot->weights = calc_sqerr(robot->particles, world->landmarks);
}

// use the particle wheel (S. Thrun)
// measurement update
void particle_resample(slambot_t *robot) {
  int i;
  double wmax;
  int index;
  double beta;
  double newparticles[robot->numparticles];
  wmax = 0.0;
  for (i = 0; i < robot->numweights; i++) {
    if (robot->weights[i] > wmax) {
      wmax = robot->weights[i];
    }
  }
  beta = 0.0;
  index = (int)(rand() * robot->numparticles) %
    robot->numparticles;
  for (i = 0; i < robot->numparticles; i++) {
    // pick a random index
    beta = beta + 2 * wmax;
    for (; beta >= robot->particles[index];
        index = (index + 1) % robot->numparticles) {
      beta -= robot->particles[index];
    }
    newparticles[i] = robot->particles[index];
  }
  memcpy(robot->particles, newparticles, sizeof(newparticles));
}

double gauss_error(double mu, double sigma) {
  double k, p;
  k = 1.0 / sqrt(2 * PI * sigma * sigma);
  p = (-(mu) / (2 * sigma * sigma));
  return k * exp(p);
}
