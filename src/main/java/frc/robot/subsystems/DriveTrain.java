package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  //Definindo Motores
  public static final int KrightMaster = 4;
  public static final int KrightSlave = 2;
  public static final int KleftMaster = 3;
  public static final int KleftSlave = 1;

  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(KrightMaster);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(KrightSlave);
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(KleftMaster);
  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(KleftSlave);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
  //Definindo Motores

  //Simulação
  private final Field2d m_field = new Field2d();

  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3),
          DCMotor.getCIM(2),
          8,
          0.6,
          0.0762,
          null
      );

  private double leftVoltage = 0;
  private double rightVoltage = 0;

  //Simulação
  
  public DriveTrain() {

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(true);
    rightSlave.setInverted(false);

    SmartDashboard.putData("Field", m_field);
}

  //Encoder
  public double getLeftPosition(){
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightPosition(){
    return rightMaster.getSelectedSensorPosition();
  }

  public double ticksToCentimeters(double ticks){
    return ticks * 0.01150390625;
  }

  public double getAvaregePosition(){
    return (ticksToCentimeters(getLeftPosition())+ticksToCentimeters(getRightPosition()))/2;
  }
  //Encoder

  //PID
  PIDController pidController = new PIDController(0.1, 0, 0);
  
  public void autoMove(double setpoint){
    pidController.setSetpoint(setpoint);
    double speed = pidController.calculate(getAvaregePosition());
    drive(speed, 0);
  }

  public void resetPosition(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }
  //PID

  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
    leftVoltage = (speed + rotation) * 6;
    rightVoltage = (speed - rotation) * 6;
  }

  public void stop() {
    diffDrive.stopMotor();
    leftVoltage = 0;
    rightVoltage = 0;
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(
        leftVoltage,
        rightVoltage
    );
    m_driveSim.update(0.02);

    m_field.setRobotPose(m_driveSim.getPose());
  }

  @Override
  public void periodic() {
    m_field.setRobotPose(m_driveSim.getPose());
  }

  public boolean isAtSetpoint() {
    throw new UnsupportedOperationException("Unimplemented method 'isAtSetpoint'");
  }
}