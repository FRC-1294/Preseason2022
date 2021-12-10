// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Spark;

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

}
