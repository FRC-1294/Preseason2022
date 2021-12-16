// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class GyroTurn extends CommandBase {
  /** Creates a new GyroTurn. */
  DriveSubsystem driveTrain;
  private boolean finish = false;
  public GyroTurn(DriveSubsystem dt, double degrees, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
    Constants.turnSpeed = speed;
    Constants.degreeTurn = degrees;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    while(driveTrain.getHeading()> Constants.degreeTurn){
      driveTrain.tankDriveVolts(-Constants.turnSpeed,Constants.turnSpeed);
    }
    while(driveTrain.getHeading()<Constants.degreeTurn){
      driveTrain.tankDriveVolts(Constants.turnSpeed, Constants.turnSpeed);
    }
    if(driveTrain.getHeading() == Constants.degreeTurn){
      finish = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
