// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  
  public ExampleSubsystem() {
    RamseteController ramsete = new RamseteController(2.0, 0.7);
    Trajectory.State goal = trajectory.sample(3.4);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  


}
