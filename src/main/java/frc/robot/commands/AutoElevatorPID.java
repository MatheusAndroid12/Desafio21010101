package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class AutoElevatorPID extends Command {

    Elevador elevador;
    double target;

    public AutoElevatorPID(Elevador elevador, double target) {
        this.elevador = elevador;
        this.target = target;
        addRequirements(elevador);
    }

    @Override
    public void initialize() {
        elevador.resetEncoders();   
        elevador.setTarget(target); 
    }

    @Override
    public boolean isFinished() {
        return elevador.inPosition();
    }

    @Override
    public void end(boolean interrupted) {
        elevador.levantagem(0);
    }
}