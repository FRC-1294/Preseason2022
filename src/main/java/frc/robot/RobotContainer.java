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
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
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