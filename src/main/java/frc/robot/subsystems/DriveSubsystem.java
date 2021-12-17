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
  private static CANSparkMax leftRearSpark = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
  private CANSparkMax rightFrontSpark = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
  private static CANSparkMax rightRearSpark = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftFrontSpark, leftRearSpark);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightFrontSpark, rightRearSpark);

  // The robot's drive
  //private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder = leftFrontSpark.getEncoder();

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder = rightFrontSpark.getEncoder();

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public XboxController driveController = new XboxController(0);

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

    arcadeDrive(-driveController.getY(Hand.kLeft), driveController.getX(Hand.kRight), driveController.getBumper(Hand.kLeft));

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

  public static void setMotors(double leftSpeed, double rightSpeed) {
    leftRearSpark.set(leftSpeed);
    rightRearSpark.set(-rightSpeed);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double forward, double turn, boolean precise) {
    final double deadZone = 0.05;
    final double minPower = 0.2;
    final double minTurn = 0.05;
    final double fastestTurn = 0.2;
    // final double maxTurnOffset = 0.1 * getSign(forward);
    
    double leftSpeed = 0;
    double rightSpeed = 0;
    
    SmartDashboard.putNumber("Input: turn", turn);

    //deadzone filter
    if (Math.abs(forward) <= deadZone) forward = 0;
    if (Math.abs(turn) <= deadZone) turn = 0;
    //precision mode
    if (precise) turn *= 0.6;

    //dynamic turn sensititvity and offset adjustments
    double turnFactor = (1-fastestTurn) * Math.pow(1-Math.pow(fastestTurn, 8/3), 6) + minTurn;
    //double turnOffset = Math.pow(forward, 2) * maxTurnOffset;

    //calculate turn correction PID
    //double turnCorrection = arcadeTurningPID.calculate(Math.abs(frontLeftSpark.getEncoder().getVelocity())-Math.abs(frontRightSpark.getEncoder().getVelocity()), 0);

    // if (turnCorrection > 1) turnCorrection = 1;
    // if (turnCorrection < -1) turnCorrection = -1;

    //apply power to inputs for higher percision at lower velocities, with applied power
    forward = ((1-minPower) * Math.abs(Math.pow(forward, 8/3)) + minPower) * getSign(forward);
    turn = ((1-minTurn) * Math.abs(Math.pow(turn, 8/3)) + minTurn) * getSign(turn) * turnFactor;// * getSign(turn);// + turnOffset;

    //differential drive logic
    leftSpeed = forward+turn;
    rightSpeed = forward-turn;

    double factor = Double.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (factor > 1) {
      factor = 1/factor;
      leftSpeed *= factor;
      rightSpeed *= factor;
    }
    
    SmartDashboard.putNumber("Output: turn", turn);
    SmartDashboard.putNumber("Output: left", leftSpeed);
    SmartDashboard.putNumber("Output: right", rightSpeed);

    //apply to PID for open loop control
    leftFrontSpark.set(leftSpeed);
    rightFrontSpark.set(rightSpeed);
    // setFrontLeftPID(maxSpeed*leftSpeed, ControlType.kVelocity, velocityPID.kSlot);
    // setFrontRightPID(maxSpeed*rightSpeed, ControlType.kVelocity, velocityPID.kSlot);
    // sparkDrive.feed();
  }
  //returns +1 or -1 based on num's sign
  private double getSign(double num) {
    double sign = num/Math.abs(num);
    if (Double.isNaN(sign)) sign = 0;

    return sign;
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
    //m_drive.feed();
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
  /*
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  */
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
