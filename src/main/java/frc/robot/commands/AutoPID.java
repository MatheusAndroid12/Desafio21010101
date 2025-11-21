
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class AutoPID extends SequentialCommandGroup {
  public AutoPID(DriveTrain drivetrain) {

    addCommands(
      new  AutoMovePID(drivetrain, 200)
    );
  }
}
