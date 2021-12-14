// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private CANSparkMax leftFrontSpark = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
  private CANSparkMax leftRearSpark = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
  private CANSparkMax rightFrontSpark = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
  private CANSparkMax rightRearSpark = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
  
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(leftFrontSpark,leftRearSpark);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(rightFrontSpark, rightRearSpark);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = leftFrontSpark.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = rightFrontSpark.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private XboxController joystick = new XboxController(0);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    initSparks();

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public void initSparks() {
    leftFrontSpark.restoreFactoryDefaults(true);
    rightFrontSpark.restoreFactoryDefaults(true);
    leftRearSpark.restoreFactoryDefaults(true);
    rightRearSpark.restoreFactoryDefaults(true);

    leftFrontSpark.getEncoder();
    rightFrontSpark.getEncoder();
    leftRearSpark.getEncoder();
    rightRearSpark.getEncoder();

    leftFrontSpark.setSmartCurrentLimit(60);
    rightFrontSpark.setSmartCurrentLimit(60);
    leftRearSpark.setSmartCurrentLimit(60);
    rightRearSpark.setSmartCurrentLimit(60);
    
    leftFrontSpark.setIdleMode(IdleMode.kBrake);
    leftRearSpark.setIdleMode(IdleMode.kBrake);
    rightFrontSpark.setIdleMode(IdleMode.kBrake);
    rightRearSpark.setIdleMode(IdleMode.kBrake);

    leftFrontSpark.setInverted(false);
    leftRearSpark.setInverted(false);
    rightFrontSpark.setInverted(true);
    rightRearSpark.setInverted(false);

    leftRearSpark.follow(leftFrontSpark);
    rightRearSpark.follow(rightFrontSpark);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(),
                      // m_rightEncoder.getPosition());

    arcadeDrive(joystick.getX(Hand.kRight), -joystick.getY(Hand.kLeft));

    SmartDashboard.putNumber("left encoder", leftFrontSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("right encoder", rightFrontSpark.getEncoder().getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
