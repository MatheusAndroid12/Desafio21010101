package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoMovePID extends Command {
  DriveTrain driveTrain;
  double setpoint;

  public AutoMovePID(DriveTrain driveTrain, double setpoint) {
    this.driveTrain = driveTrain;
    this.setpoint = setpoint;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.resetPosition();
  }

  @Override
  public void execute() {
    driveTrain.autoMove(setpoint);
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