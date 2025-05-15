#include <random>
#include "random.hpp"

float uniform_random() {
  static thread_local std::minstd_rand gen(std::random_device{}());
  static thread_local std::uniform_real_distribution<float> dist(0.0, 1.0);
  return dist(gen);
}