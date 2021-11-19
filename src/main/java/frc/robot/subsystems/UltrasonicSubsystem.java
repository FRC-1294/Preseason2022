// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UltrasonicSubsystem extends SubsystemBase {
  /** Creates a new UltrasonicSubsystem. */

  Ultrasonic UltraSensor;  

  public UltrasonicSubsystem() {

    UltraSensor = new Ultrasonic(Constants.UltraSensorPingChannel,Constants.UltraSensorEchoChannel); //TODO: Get correct numbers
    UltraSensor.setEnabled(true);
  }

  

  public double getInches() {
    return UltraSensor.getRangeInches();

  }

  public double getMM() {
    return UltraSensor.getRangeMM();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
