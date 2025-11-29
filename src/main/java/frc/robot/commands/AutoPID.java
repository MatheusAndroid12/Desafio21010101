package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Shooter;

public class AutoPID extends SequentialCommandGroup {

    public AutoPID(DriveTrain drivetrain, Elevador elevador, Shooter shooter) {

        addCommands(
            new AutoMovePID(drivetrain, 150),       
            new AutoSubida(elevador, 11 ),      
            new AutoAtiragem(shooter, 1.0),
            new AutoSubida(elevador, 0)       
        );
    }
}