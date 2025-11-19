package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;


public class PIDAuto extends SequentialCommandGroup {
  public PIDAuto(DriveTrain driveTrain) {
    addCommands(
      new AutoMovePID(driveTrain,200),
      new AutoRotatePID (driveTrain, 90),
      new AutoMovePID(driveTrain, 300)
    );
  }
}