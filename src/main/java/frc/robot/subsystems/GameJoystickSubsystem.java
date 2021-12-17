// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GameJoystickSubsystem extends SubsystemBase {
  /** Creates a new GameJoystickSubsystem. */
  TalonSRX rightIntake = new TalonSRX(21);
  TalonSRX leftIntake = new TalonSRX(23);
  DriveSubsystem drive;

  public GameJoystickSubsystem(DriveSubsystem drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    rightIntake.set(TalonSRXControlMode.PercentOutput, (drive.driveController.getTriggerAxis(Hand.kLeft)-drive.driveController.getTriggerAxis(Hand.kRight)));
    leftIntake.set(TalonSRXControlMode.PercentOutput, 0.3*(drive.driveController.getTriggerAxis(Hand.kLeft)-drive.driveController.getTriggerAxis(Hand.kRight)));
  }
}
