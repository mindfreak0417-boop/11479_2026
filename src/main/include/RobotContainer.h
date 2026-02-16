// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "subsystems/SpeedControl.h"
#include <ctre/phoenix6/controls/MotionMagicDutyCycle.hpp>


class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric FieldCentric_Manualdrive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);
    swerve::requests::FieldCentricFacingAngle FieldCentricFacingAngle_Manualdrive = swerve::requests::FieldCentricFacingAngle{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity).WithHeadingPID(8, 0, 0); // Use open-loop control for drive motors


    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};

    /////////////////////////////////////
    // ctre::phoenix6::CANBus kCANBus{"rio"};

    ctre::phoenix6::hardware::TalonFX m_fx{51};
    ctre::phoenix6::hardware::TalonFX m_fx1{52};
    ctre::phoenix6::hardware::TalonFX m_fx2{54};

    /* Start at velocity 0, use slot 1 */
    // ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocityTorque =
    //     ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps}.WithSlot(1);
    ctre::phoenix6::controls::VelocityDutyCycle m_request =ctre::phoenix6::controls::VelocityDutyCycle{0_tps}.WithSlot(1);
    /* Keep a neutral out so we can disable the motor */
    ctre::phoenix6::controls::NeutralOut m_brake{};
    /////////////////////////////////////


public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> autoChooser;

public:
    RobotContainer();
    frc2::Command *GetAutonomousCommand();
    frc::Field2d m_Field2d;  
    frc::Translation2d TargetTranslation{}; 
    SpeedControl test_speedControl1{51};
    SpeedControl test_speedControl2{52};



private:
    void ConfigureBindings();
};
