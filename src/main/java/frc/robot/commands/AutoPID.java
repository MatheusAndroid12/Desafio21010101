package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Shooter;

public class AutoPID extends SequentialCommandGroup {

    public AutoPID(DriveTrain drivetrain, Elevador elevador, Shooter shooter) {

        addCommands(
            new AutoMovePID(drivetrain, -2),       
            new AutoSubida(elevador, 13),      
            new AutoAtiragem(shooter, 2.0)           
        );
    }
}