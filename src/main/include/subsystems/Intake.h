// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "subsystems/modules/DualMotorModule.h"
#include "subsystems/modules/SingleMotorModule.h"

using namespace ctre::phoenix6;
using TPS = units::turns_per_second_t;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

 private:

};
