package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoMovePID extends Command {
  DriveTrain driveTrain;
  double distancia;

  public AutoMovePID(DriveTrain driveTrain, double distancia) {
    this.driveTrain = driveTrain;
    this.distancia = distancia;
  }

  @Override
  public void initialize() {
    driveTrain.resetPosition();
  }

  @Override
  public void execute() {
    driveTrain.autoMove(distancia);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(1, 0);
  }

  @Override
  public boolean isFinished() {
    return driveTrain.isAtSetpoint();
  }
}