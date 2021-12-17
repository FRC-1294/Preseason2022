// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.subsystems.UltrasonicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForwardCMD;
import frc.robot.commands.GyroTurn;
import frc.robot.commands.Trajectory_Command;
import frc.robot.subsystems.DriveSubsystem;



import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.VisionSubsystem;
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

  public final UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem();

  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final VisionCommand m_visionCommand = new VisionCommand(m_visionSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = new DriveSubsystem();
    driveForwardCMD = new DriveForwardCMD(drive, 1, 0.5);
    driveForwardCMD.addRequirements(drive);
    gyroTurnCMD = new GyroTurn(drive, 135, 0.5);

    // Configure the button bindings
    configureButtonBindings();

    m_visionCommand.schedule(true);
  }
  XboxController joystick = new XboxController(0); 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // A = 1
    // B = 2
    // X = 3
    // Y = 4
    // Left Bumper = 5
    // Right Bumper = 6
    // Back = 7
    // Start = 8
    // Left Stick = 9
    // Right Stick = 10

    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    new SequentialCommandGroup(new GyroTurn(drive, 135, 0.5), new DriveForwardCMD(drive, 1, 0.5), new GyroTurn(drive, 135, 0.5), new DriveForwardCMD(drive, 1, 0.5), new GyroTurn(drive, 135, 0.5));
    return new Trajectory_Command(drive).getAutonomousCommand();
    
  }
//buttons 
  public boolean aButtonPressed() {
    return joystick.getAButton(); 
  }
  public boolean bButtonPressed(){ 
    return joystick.getBButton();
  }
  public boolean xButtonPressed(){
    return joystick.getXButton(); 
  }
  public boolean yButtonPressed(){ 
    return joystick.getYButton(); 
    //return m_visionCommand;
  }

  //joysticks
  public double leftJoystickY() {
    return joystick.getY(Hand.kLeft);
  } 
  public double leftJoystickX(){ 
    return joystick.getX(Hand.kRight);
  }


  public double LeftTrigger() { 
    return joystick.getTriggerAxis(Hand.kLeft);
  }

  public double RightTrigger(){ 
    return joystick.getTriggerAxis(Hand.kRight);
  }


}