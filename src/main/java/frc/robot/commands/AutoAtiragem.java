package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoAtiragem extends Command {

    Shooter shooter;
    Timer timer = new Timer();
    double tempo;

    public AutoAtiragem(Shooter shooter, double tempo) {
        this.shooter = shooter;
        this.tempo = tempo;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.run(-1.0); 
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= tempo;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.run(0);
    }
}