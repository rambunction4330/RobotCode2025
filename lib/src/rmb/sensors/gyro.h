#pragma once

#include "frc/geometry/Rotation2d.h"
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>

namespace rmb {
class Gyro {
public:
  virtual frc::Rotation2d getRotation() const = 0;
  virtual frc::Rotation2d getRotationNoOffset() const = 0;

  /**
   * Sets offset to zero, sets gyro rotation to zero
   */
  virtual void resetZRotation() = 0;

  virtual units::meters_per_second_squared_t getXAcceleration() const = 0;
  virtual units::meters_per_second_squared_t getYAcceleration() const = 0;
  virtual units::meters_per_second_squared_t getZAcceleration() const = 0;

  virtual units::meters_per_second_t getXVelocity() const = 0;
  virtual units::meters_per_second_t getYVelocity() const = 0;
  virtual units::meters_per_second_t getZVelocity() const = 0;

  void setZRotationOffset(frc::Rotation2d offset) { this->offset = offset; }

protected:
  frc::Rotation2d offset = 0_deg;
};
} // namespace rmb
