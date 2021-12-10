package frc.robot.subsystems;

/* >>------>
           ___
          |_|_|
          |_|_|              _____
          |_|_|     ____    |*_*_*|
 _______   _\__\___/ __ \____|_|_   _______
/ ____  |=|      \  <_+>  /      |=|  ____ \
~|    |\|=|======\\______//======|=|/|    |~
 |_   |    \      |      |      /    |    |
  \==-|     \     | 1294 |     /     |----|~~/
  |   |      |    |      |    |      |____/~/
  |   |       \____\____/____/      /    / /
  |   |         {----------}       /____/ /
  |___|        /~~~~~~~~~~~~\     |_/~|_|/
   \_/        |/~~~~~||~~~~~\|     /__|\
   | |         |    ||||    |     (/|| \)
   | |        /     |  |     \       \\
   |_|        |     |  |     |
              |_____|  |_____|
              (_____)  (_____)
              |     |  |     |
              |     |  |     |
              |/~~~\|  |/~~~\|
              /|___|\  /|___|\
             <_______><_______>

  --Tune PID
  --Test it
<------<< */

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoxCollector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public BoxCollector() {}
  final double kP = 0.5;
  final double kI = 0;
  final double kD = 0.5;
  PIDController pid = new PIDController(kP, kI, kD);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
  double setpoint = 0;
  public void pullBox() {
      double in = UltrasonicSubsystem.getInches();
      
      if (in < Constants.UltraSensorThreshold) {
        setpoint = in * 12;
        double sensorPosition = encoder.get() * kDriveTick2Feet;
        double error = setpoint - sensorPosition;
        double outputSpeed = kP * error;

        Constants.armMotor1.set(outputSpeed);
        Constants.armMotor2.set(-outputSpeed);
      }
  }
}
