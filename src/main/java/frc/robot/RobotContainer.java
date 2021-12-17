// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForwardCMD;
import frc.robot.commands.DriveForwardEncoder;
import frc.robot.commands.GyroTurn;
import frc.robot.commands.Trajectory_Command;
import frc.robot.subsystems.DriveSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem drive;
  private DriveForwardCMD driveForwardCMD;
  private GyroTurn gyroTurnCMD;
  private DriveForwardEncoder driveforwardEncoder;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = new DriveSubsystem();
    driveForwardCMD = new DriveForwardCMD(drive, 1, 0.5);
    driveForwardCMD.addRequirements(drive);
    gyroTurnCMD = new GyroTurn(drive, 135, 0.5);
    driveforwardEncoder = new DriveForwardEncoder(drive, 0, 0);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    new SequentialCommandGroup(
      new DriveForwardEncoder(drive, 1.7526, 0.5),
      new GyroTurn(drive, 135, 0.5),
      new DriveForwardEncoder(drive, 2.6416, 0.5), 
      new GyroTurn(drive, 135, 0.5), 
      new DriveForwardEncoder(drive, 2.032, 0.5), 
      new GyroTurn(drive, 135, 0.5));
      new DriveForwardEncoder(drive, 1.7526, 0.5);
    return new Trajectory_Command(drive).getAutonomousCommand();
    
  }
}
