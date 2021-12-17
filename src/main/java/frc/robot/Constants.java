// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants { 

    //TODO: Double check these
    public static int UltraSensorPingChannel = 0;
    public static int UltraSensorEchoChannel = 1;
    public static double UltraSensorThreshold = 10;
    public static Spark armMotor1 = new Spark(0);
    public static Spark armMotor2 = new Spark(1);

    public static int xboxControllerPort = 9480;
    public static XboxController xboxController = new XboxController(xboxControllerPort);

    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final class DriveConstants {
        public static final int kLeftMotorPort = 0;
        public static final int kRightMotorPort = 1;
        public static final int kLeftEncoderChannelA = 0;
        public static final int kLeftEncoderChannelB = 1;
        public static final int kRightEncoderChannelA = 2;
        public static final int kRightEncoderChannelB = 3;
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }


    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.001;
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double DriveConstants = 1.5;
    //need to update left and right ports below
    public static final int kLeftMotor1Port = 10;
    public static final int kLeftMotor2Port = 12;
    public static final int kRightMotor1Port = 11;
    public static final int kRightMotor2Port = 13;
    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;
    }

    //need to update left and right encoder ports as well
    public static final int[] kRightEncoderPorts= {0};
    public static final boolean kRightEncoderReversed= false;
    public static final int[] kLeftEncoderPorts= {0};
    public static final boolean  kLeftEncoderReversed= false;
    public static final double kEncoderDistancePerPulse = 0;
}    
