

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

public class SubirManual extends Command {
  private Elevador elevador;
  private Joystick controller;
    public SubirManual(Elevador elevador, Joystick controller) {
      this.elevador = elevador;
      this.controller = controller;
      addRequirements(elevador);
    }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(controller.getPOV() == 0){
      elevador.run(-0.25);

    }
    else if(controller.getPOV()== 180){
      elevador.run(0.25);
    }
    else{
      elevador.run(0);
    }
  }


  @Override
  public void end(boolean interrupted) {
    elevador.run(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
