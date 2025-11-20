package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class CoralAutonomo extends SequentialCommandGroup {
    public CoralAutonomo(DriveTrain drivetrain, Elevador elevador, Shooter shooter) {
        addCommands(
            new AutoMovePID(drivetrain, 200),
            new AutoRotatePID(drivetrain, 90),
            new WaitCommand(0.1),
            new Command() {
                @Override
                public void initialize() {
                    elevador.setTarget(18);
                }

                @Override
                public void execute() {
                    elevador.elevatorPIDMove(elevador.getTarget());
                }

                @Override
                public void end(boolean interrupted) {
                    elevador.levantagem(0);
                }

                @Override
                public boolean isFinished() {
                    return elevador.inPosition();
                }
            },
            new Command() {
                private double startTime;

                @Override
                public void initialize() {
                    startTime = Timer.getFPGATimestamp();
                }

                @Override
                public void execute() {
                    shooter.run(0.5);
                }

                @Override
                public void end(boolean interrupted) {
                    shooter.run(0);
                }

                @Override
                public boolean isFinished() {
                    return Timer.getFPGATimestamp() - startTime > 1.5;
                }
            }
        );
    }
}
