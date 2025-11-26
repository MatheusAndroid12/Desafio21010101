package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoMovePID extends Command {
  DriveTrain driveTrain;
  double setpointLeft;
  double setpointRight;

  public AutoMovePID(DriveTrain driveTrain, double setpointLeft, double setpointRight) {
    this.driveTrain = driveTrain;
    this.setpointRight = setpointRight;
    this.setpointLeft = setpointLeft;

  }

  @Override
  public void initialize() {
    driveTrain.resetPosition();
  }

  @Override
  public void execute() {
    driveTrain.autoMove(setpointLeft, setpointRight);
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