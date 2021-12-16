// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArcadeDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax frontLeftSpark = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
  private CANSparkMax frontRightSpark = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
  private CANSparkMax rearLeftSpark = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
  private CANSparkMax rearRightSpark = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);

  SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftSpark, rearLeftSpark);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightSpark, rearRightSpark);

  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  public XboxController driveController = new XboxController(0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    ArcadeDrive drive = new ArcadeDrive(this, m_drive);
    drive.schedule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

