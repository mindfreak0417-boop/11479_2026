#include "subsystems/SpeedControl.h"

#include <iostream>

using ctre::phoenix::StatusCode;

SpeedControl::SpeedControl(int canID)
    : m_fx{canID} 
{
  m_request = m_request.WithSlot(1);

  ctre::phoenix6::configs::TalonFXConfiguration configs{};

  configs.Slot1.kS = 0.0;
  configs.Slot1.kP = 0.0335;
  configs.Slot1.kI = 0.0;
  configs.Slot1.kD = 0.0;

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40_A;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40_A;

  StatusCode status = StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_fx.GetConfigurator().Apply(configs);
    if (status.IsOK()) break;     
  }
  if (!status.IsOK()) {
    std::cout << "Failed to apply TalonFX configs: " << status.GetName()
              << std::endl;
  }

  Stop();
}


void SpeedControl::SetSpeed(units::angular_velocity::turns_per_second_t tps) {
  auto status = m_fx.SetControl(m_request.WithVelocity(5_tps));
  if (!status.IsOK()) {
    std::cout << "SetControl failed: " << status.GetName() << "\n";
  } else {
    std::cout << "Set speed to " << tps.value() << " turns per second\n";
  }
}

void SpeedControl::Stop() {
  auto status = m_fx.SetControl(m_brake);
  if (!status.IsOK()) {
    std::cout << "NeutralOut failed: " << status.GetName() << "\n";
  }
}

frc2::CommandPtr SpeedControl::ExampleMethodCommand(units::turns_per_second_t rps) {
  return frc2::cmd::Sequence(
    frc2::cmd::Run([this] {
        SetSpeed(2_tps);
    }, {this}).WithTimeout(2_s),

    frc2::cmd::Wait(1_s),

    frc2::cmd::RunOnce([this] {
      Stop();
    }, {this})
);
}

