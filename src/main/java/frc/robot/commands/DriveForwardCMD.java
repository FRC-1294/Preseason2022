package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class DriveForwardCMD extends CommandBase {
  /** Creates a new DriveForwardCMD. */
  DriveSubsystem driveTrain;
  private boolean finish = false;
  Timer timer;
  public DriveForwardCMD(DriveSubsystem dt, double time, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
    timer = new Timer();
    Constants.driveTime = time;
    Constants.autonomousSpeed = speed;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while (timer.get() < Constants.driveTime) {
      driveTrain.tankDriveVolts(Constants.autonomousSpeed, Constants.autonomousSpeed);
    }
    finish = true;
    System.out.println("DriveForwardCmd started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return finish;
  }
}