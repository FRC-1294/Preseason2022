// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
public class DriveForwardEncoder extends CommandBase {
  DriveSubsystem driveTrain;
  private boolean finish = false;
  public DriveForwardEncoder(DriveSubsystem dt, double distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
    Constants.distance = distance;
    Constants.speedForEncoder = speed;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    while(DriveSubsystem.getEncoderMeters() < Constants.distance) {
      driveTrain.tankDriveVolts(Constants.speedForEncoder, Constants.speedForEncoder);
    }
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
