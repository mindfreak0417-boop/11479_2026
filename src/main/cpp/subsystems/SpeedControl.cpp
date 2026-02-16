#include "subsystems/SpeedControl.h"

//#include <ctre/phoenix6/controls/Follower.hpp>

#include <iostream>

using ctre::phoenix::StatusCode;

SpeedControl::SpeedControl() {
  m_velTorque = m_velTorque.WithSlot(1);

  ctre::phoenix6::configs::TalonFXConfiguration configs{};

  configs.Slot1.kS = 2.5;  
  configs.Slot1.kP = 5.0;  
  configs.Slot1.kI = 0.0;
  configs.Slot1.kD = 0.0;

  // Torque current limits
  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40_A;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40_A;

  // Apply configs with retry
  StatusCode status = StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_fx.GetConfigurator().Apply(configs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply TalonFX configs: " << status.GetName()
              << std::endl;
  }
  
  Stop();
}

void SpeedControl::SetSpeed(units::turns_per_second_t rps) {
  m_fx.SetControl(m_velTorque.WithVelocity(rps));
}

void SpeedControl::Stop() {
  m_fx.SetControl(m_neutral);
}
