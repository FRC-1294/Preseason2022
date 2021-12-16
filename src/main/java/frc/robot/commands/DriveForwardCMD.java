// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCMD extends CommandBase {
  private double distance;

  public void DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
    this.distance = DriveSubsystem.getEncoderMeters() + distance;
    addRequirements(driveSubsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardCmd started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.setMotors(DriveConstants.kAutoDriveForwardSpeed, DriveConstants.kAutoDriveForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.setMotors(0, 0);
        System.out.println("DriveForwardCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriveSubsystem.getEncoderMeters() > distance)
            return true;
        else
            return false;
  }
}
