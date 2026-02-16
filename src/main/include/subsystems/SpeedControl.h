#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANBus.hpp>
#include "ctre/phoenix6/TalonFX.hpp"
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>

class SpeedControl : public frc2::SubsystemBase {
 public:
  SpeedControl();
    
  void SetSpeed(units::turns_per_second_t rps);
  void Stop();

 private:
  ctre::phoenix6::CANBus m_canbus{"canivore"};

  ctre::phoenix6::hardware::TalonFX m_fx{0, m_canbus};

  // torque-based velocity request (Slot1)
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velTorque{0_tps};
  ctre::phoenix6::controls::NeutralOut m_neutral{};
};
