package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ComandoShooter extends Command{
    private Shooter shooter;

    public ComandoShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override 

    public void initialize() {
    }
    @Override

    public void execute() {
        shooter.run(0.5);
    }
    @Override

    public void end (boolean interrupted) {
        shooter.run(0);
    }
    @Override 

    public boolean isFinished() {
        return false;
    }
}