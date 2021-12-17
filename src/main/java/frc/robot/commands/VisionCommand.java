// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//limelight stuff
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final VisionSubsystem m_subsystem;
    private NetworkTable table;
    private double targetOffsetAngle_Horizontal;
    private double targetOffsetAngle_Vertical;
    private final double maxTarget = 69;
    private final double minTarget = 420;
    private double targetArea;
    private boolean isFinished;
    private DriveSubsystem vromVrom;

  public VisionCommand(VisionSubsystem subsystem) {
    m_subsystem = subsystem;
   
    addRequirements(subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0.0);
      targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);
      targetArea = table.getEntry("ta").getDouble(0.0);

      System.out.println("horizontal target offset angle: " + targetOffsetAngle_Horizontal);
      System.out.println("vertical target offset angle: " + targetOffsetAngle_Horizontal);
      System.out.println("area: " + targetArea);

      if (targetOffsetAngle_Vertical < maxTarget && targetOffsetAngle_Vertical > minTarget) {
        isFinished = true;
      } else {
        vromVrom.arcadeDrive(.19, 0, false);
        System.out.println("vrom vrom");
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vromVrom.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
