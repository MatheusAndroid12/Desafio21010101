package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class ComandoDriveTrain extends Command {
    private  DriveTrain drivetrain;
    private Joystick controller;

    public ComandoDriveTrain(Joystick controller, DriveTrain drivetrain){
        this.controller = controller;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize(){}

    public void execute(){
        drivetrain.drive(controller.getZ(), controller.getY());
    }   

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return false;
    }
}