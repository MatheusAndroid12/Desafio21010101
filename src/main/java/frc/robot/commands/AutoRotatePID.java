package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoRotatePID extends Command {

    DriveTrain drivetrain;
    PIDController turnPID = new PIDController(0.01, 0, 0);
    double target;

    public AutoRotatePID(DriveTrain drivetrain, double graus) {
        this.drivetrain = drivetrain;
        this.target = graus;
        turnPID.setTolerance(2);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetPosition(); 
    }

    @Override
    public void execute() {
        double rotation = turnPID.calculate(drivetrain.getAvaregePosition(), target);
        drivetrain.drive(0, rotation);
    }

    @Override
    public boolean isFinished() {
        return turnPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0);
    }
}