package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final int Kid = 5;

    double speed = 0.5;
    boolean turbo = true;

    final TalonSRX RodaOuttake; 
  
      public Shooter() {
       
        RodaOuttake = new TalonSRX(Kid);

      }

    public void run(double speed) {
      RodaOuttake.set(TalonSRXControlMode.PercentOutput, speed);

    }
    @Override
    public void periodic() {
    }
}