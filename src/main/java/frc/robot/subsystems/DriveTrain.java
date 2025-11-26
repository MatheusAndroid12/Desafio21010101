package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  public static final int KrightMaster = 4;
  public static final int KrightSlave = 2;
  public static final int KleftMaster = 3;
  public static final int KleftSlave = 1;

  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(KrightMaster);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(KrightSlave);
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(KleftMaster);
  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(KleftSlave);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

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
  
  PIDController pidController = new PIDController(0.05, 0, 0);


  public DriveTrain() {

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.setSensorPhase(true);
    
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave.setInverted(false);

    SmartDashboard.putData("Field", m_field);

    pidController.setTolerance(10);

    resetPosition();
}

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

  
  public void autoMove(double setpoint){
    pidController.setSetpoint(setpoint);
    double speed = pidController.calculate(getAvaregePosition());
    drive(-speed, 0);

    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("SetPoint", setpoint);
  }


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
    // m_field.setRobotPose(m_driveSim.getPose());

    SmartDashboard.putNumber("LeftPosition", getLeftPosition());
    SmartDashboard.putNumber("RightPosition", getRightPosition());

    SmartDashboard.putNumber("AvaragePosition", getAvaregePosition());
    SmartDashboard.putBoolean("in the setpoint2", pidController.atSetpoint());
    SmartDashboard.putNumber("LeftMaster", leftMaster.get());
    SmartDashboard.putNumber("RightMaster", rightMaster.get());
    SmartDashboard.putNumber("LeftSlave", leftSlave.get());
    SmartDashboard.putNumber("RigtSlave", rightSlave.get());
  }

  public boolean isAtSetpoint() {
    return pidController.atSetpoint();
  }

  public void resetPosition(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

}