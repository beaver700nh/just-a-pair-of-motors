// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <rev/CANSparkMax.h>

using MotorType = rev::CANSparkMaxLowLevel::MotorType;

struct MotorThing {
  rev::CANSparkMax m_motor;
  double m_power;
  const double m_ramp;

  void set_power(double power);
};

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  frc::XboxController m_controller {0};
  MotorThing m_extender {rev::CANSparkMax {7, MotorType::kBrushless}, 0.0, 0.05};
  MotorThing m_pivoter {rev::CANSparkMax {8, MotorType::kBrushless}, 0.0, 0.05};
};
