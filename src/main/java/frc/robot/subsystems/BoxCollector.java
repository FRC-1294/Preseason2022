package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

/* >>------>

>>--> Schrödinger's Code <--<<
  ,-.       _,---._ __  / \
 /  )    .-'       `./ /   \
(  (   ,'            `/    /|
 \  `-"             \'\   / |
  `.              ,  \ \ /  |
   /`.          ,'-`----Y   |
  (            ;        |   '
  |  ,-.    ,-'         |  /
  |  | (   |            | /
  )  |  \  `.___________|/
  `--'   `--'
--[ You don't know if it works until you test it ]--

<------<< */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoxCollector extends SubsystemBase {

  public static Spark armMotor1 = new Spark(0);
  public static Spark armMotor2 = new Spark(1);

  public static boolean overriding = false;

  private boolean cancelOveride = false;
  private double instantManualEmergencyRobotFullSystemOverideSparkMotorConstantSpeedAsDoubleSubjectToChange = 1;

  @Override
  public void periodic() {
    if (Constants.xboxController.getAButton()) {
      overide(instantManualEmergencyRobotFullSystemOverideSparkMotorConstantSpeedAsDoubleSubjectToChange);
    }
    else if (cancelOveride) {
      overide(0);
      cancelOveride = false;
    } else {
      cancelOveride = true;

      autoCollector();
    }
  }

  public void autoCollector() {
    if (overriding) return;

    double in = UltrasonicSubsystem.getInches();
      
    if (in < Constants.UltraSensorThreshold) {
      if (in >= 0.5) {
        armMotor1.set(1);
        armMotor2.set(-1);
      } else {
        armMotor1.set(0);
        armMotor2.set(0);
      }
    }
    
    }

  public static void overide(double speed) {
    armMotor1.set(speed);
    armMotor2.set(-speed);
}
}

