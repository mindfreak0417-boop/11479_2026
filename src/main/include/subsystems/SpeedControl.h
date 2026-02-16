#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <ctre/phoenix6/CANBus.hpp>
#include "ctre/phoenix6/TalonFX.hpp"
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
// #include <ctre/phoenix6/configs/TalonFXConfiguration.hpp>


class SpeedControl : public frc2::SubsystemBase {
 public:
  SpeedControl(int canID);
  
  frc2::CommandPtr ExampleMethodCommand(units::turns_per_second_t rps);

  void SetSpeed(units::angular_velocity::turns_per_second_t tps);
  void Stop();

 private:
  ctre::phoenix6::hardware::TalonFX m_fx;
  
  ctre::phoenix6::controls::VelocityVoltage m_request =ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(1);
    /* Keep a neutral out so we can disable the motor */
  ctre::phoenix6::controls::NeutralOut m_brake{};
};


