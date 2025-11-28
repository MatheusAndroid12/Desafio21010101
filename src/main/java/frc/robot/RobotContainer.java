package frc.robot;

import frc.robot.commands.AutoMovePID;
import frc.robot.commands.AutoPID;
import frc.robot.commands.AutoSubida;
import frc.robot.commands.ComandoDriveTrain;
import frc.robot.commands.ComandoShooter;
import frc.robot.commands.ComandoShooterNegativo;
import frc.robot.commands.SubirManual;
import frc.robot.commands.ComandoElevador;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevador;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    public Elevador elevador = new Elevador();
  
    private static final int Kcontroller = 0;

    private Joystick controller = new Joystick(Kcontroller);

    private DriveTrain drivetrain = new DriveTrain(); 
    private Shooter shooter = new Shooter();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new ComandoDriveTrain(controller, drivetrain));
        elevador.setDefaultCommand(new SubirManual(elevador, controller));
        


        new JoystickButton(controller, 1).whileTrue(new ComandoShooter(shooter));
        new JoystickButton(controller, 2).whileTrue(new ComandoShooterNegativo(shooter));
    }

    public Command getAutonomousCommand() {
        return new AutoSubida(elevador, 6.8);
      }
}