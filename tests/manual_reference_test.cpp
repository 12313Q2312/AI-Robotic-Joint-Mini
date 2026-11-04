#include <cassert>
#include <cmath>
#include <iostream>
#include "ai_robotic_joint_ros2/ManualReference.hpp"

int main() {
  using ai_robotic_joint_ros2::ManualReference;
  constexpr double rad = 3.14159265358979323846 / 180.0;
  ManualReference reference(3 * rad, 5 * rad, -45 * rad, 45 * rad);
  reference.reset(0.0);
  reference.moveTo(30 * rad);
  double previous_position = 0;
  double previous_velocity = 0;
  for (int i = 0; i < 1600; ++i) {
    reference.step(0.01);
    assert(reference.position() >= previous_position - 1e-10);
    assert(reference.position() <= 30 * rad + 1e-10);
    assert(std::abs(reference.velocity()) <= 3 * rad + 1e-10);
    assert(std::abs(reference.velocity() - previous_velocity) <= 5 * rad * 0.01 + 1e-9);
    previous_position = reference.position();
    previous_velocity = reference.velocity();
  }
  assert(std::abs(reference.position() - 30 * rad) < 1e-9);
  assert(std::abs(reference.velocity()) < 1e-9);
  std::cout << "PASS slow angle reference, velocity and acceleration limits\n";
  // Operator releases while moving: bounded deceleration, no continued goal.
  reference.reset(0); reference.moveTo(45 * rad);
  for (int i = 0; i < 200; ++i) reference.step(0.01);
  const double released_at = reference.position();
  reference.stop();
  for (int i = 0; i < 200; ++i) reference.step(0.01);
  assert(reference.position() < released_at + 1 * rad);
  assert(reference.position() < 45 * rad);
  assert(!reference.moving());
  // Single 0.25 degree command; reversal and many changing operator goals.
  reference.reset(0); reference.moveTo(0.25 * rad);
  for (int i = 0; i < 200; ++i) reference.step(0.01);
  assert(std::abs(reference.position() - 0.25 * rad) < 1e-10);
  reference.moveTo(-45 * rad);
  for (int i = 0; i < 1900; ++i) reference.step(0.01);
  assert(std::abs(reference.position() + 45 * rad) < 1e-10);
  for (int trial = 0; trial < 200; ++trial) {
    reference.moveTo(((trial * 37) % 91 - 45) * rad);
    for (int i = 0; i < 73; ++i) {
      const double old = reference.velocity();
      reference.step(0.013);
      assert(std::abs(reference.velocity()) <= 3 * rad + 1e-9);
      assert(std::abs(reference.velocity() - old) <= 5 * rad * 0.013 + 1e-8);
      assert(std::abs(reference.position()) <= 45 * rad + 1e-9);
    }
    if (trial % 3 == 0) reference.stop();
  }
  bool rejected = false;
  try { reference.moveTo(46 * rad); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  rejected = false;
  try { reference.step(0); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  std::cout << "PASS release, fine step, reversals, 200 retargets, invalid commands\n";
}
