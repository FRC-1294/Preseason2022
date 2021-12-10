package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BoxCollector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Spark armMotor1 = new Spark(0);
  public static Spark armMotor2 = new Spark(1);

  @Override
  public void periodic() {
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
}
