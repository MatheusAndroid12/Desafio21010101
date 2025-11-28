package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class ComandoElevador extends Command {

    Elevador elevador;
    Joystick controller;
    double target;
    
    public ComandoElevador(Elevador elevador, Joystick controller, double target) {
        this.elevador = elevador;
        this.controller = controller;
        this.target = target;
        addRequirements(elevador);
    }

    @Override
    public void execute() {
        switch (controller.getPOV()) {
            case 0 -> elevador.setTarget(0);
            case 180 -> elevador.setTarget(11);
            case 90 -> elevador.setTarget(5.6);
            case 270 -> elevador.setTarget(12);
            }  
            // elevador.elevatorPIDMove(elevador.getTarget());
        }


    @Override
    public void end(boolean interrupted) {
        elevador.levantagem(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}