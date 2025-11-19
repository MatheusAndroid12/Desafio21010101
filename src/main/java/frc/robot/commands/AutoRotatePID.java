package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoRotatePID extends Command {
  DriveTrain driveTrain;
  double rotation;

  public AutoRotatePID(DriveTrain driveTrain, double rotation) {
    this.driveTrain = driveTrain;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    driveTrain.resetPosition();
  }

  @Override
  public void execute() {
    driveTrain.autoMove(rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return driveTrain.isAtSetpoint();
  }
}