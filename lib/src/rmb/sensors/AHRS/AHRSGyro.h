#pragma once

#include "studica/AHRS.h"
#include "frc/SerialPort.h"
#include "frc/geometry/Rotation2d.h"
#include "studica/AHRS.h"
#include "units/acceleration.h"
#include <memory>
#include <rmb/sensors/gyro.h>

namespace rmb {
class NavXGyro : public Gyro {
public:
  NavXGyro(studica::AHRS::NavXComType port);

  virtual ~NavXGyro() = default;

  virtual void resetZRotation() override;

  virtual frc::Rotation2d getRotationNoOffset() const override;
  virtual frc::Rotation2d getRotation() const override;

  virtual units::meters_per_second_squared_t getXAcceleration() const override;
  virtual units::meters_per_second_squared_t getYAcceleration() const override;
  virtual units::meters_per_second_squared_t getZAcceleration() const override;

  virtual units::meters_per_second_t getXVelocity() const override;
  virtual units::meters_per_second_t getYVelocity() const override;
  virtual units::meters_per_second_t getZVelocity() const override;

private:
  std::unique_ptr<studica::AHRS> gyro;
};
} // namespace rmb
