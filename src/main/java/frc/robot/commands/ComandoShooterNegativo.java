package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ComandoShooterNegativo extends Command {
    private Shooter shooter;

    public ComandoShooterNegativo (Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    } 
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.run(-1);

    }
    @Override
    public void end(boolean interrupted) {
        shooter.run(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}