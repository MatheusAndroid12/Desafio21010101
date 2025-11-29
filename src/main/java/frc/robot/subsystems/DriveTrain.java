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

  // IDs dos motores no CAN
  public static final int KrightMaster = 4;
  public static final int KrightSlave = 2;
  public static final int KleftMaster = 3;
  public static final int KleftSlave = 1;

  // Motores principais
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(KrightMaster);
  private final WPI_TalonSRX rightSlave = new WPI_TalonSRX(KrightSlave);
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(KleftMaster);
  private final WPI_TalonSRX leftSlave = new WPI_TalonSRX(KleftSlave);

  
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  // Campo para SmartDashboard (simulação)
  private final Field2d m_field = new Field2d();

  // Simulador do drivetrain (para quando estiver usando simulação)
  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(
              1.98, // ks
              0.2,  // kv
              1.5,  // ka
              0.3   // j
          ),
          DCMotor.getCIM(2),
          8,        // redução
          0.6,      // track width
          0.0762,   // raio da roda
          null
      );

  // Tensões aplicadas nos motores (usado na simulação)
  private double leftVoltage = 0;
  private double rightVoltage = 0;
  
  // Controlador PID para o movimento autônomo
  PIDController pidController = new PIDController(0.5, 0, 0);

  // Construtor
  public DriveTrain() {

    // Configura sensores de encoders nos masters
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    // Fase correta do sensor direito
    rightMaster.setSensorPhase(true);
    
    // Configura slaves seguindo masters
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    // Inversões dos motores
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave.setInverted(false);

    // Publica o campo no dashboard
    SmartDashboard.putData("Field", m_field);

    // Tolerância para considerar estar no setpoint
    pidController.setTolerance(5);

    // Zera os encoders ao iniciar
    resetPosition();

    // Limita a velocidade máxima do arcade drive
    diffDrive.setMaxOutput(0.7);
  }

  // Métodos de sensores
  public double getLeftPosition() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return rightMaster.getSelectedSensorPosition();
  }

  // Converte ticks para centímetros
  public double ticksToCentimeters(double ticks) {
    return ticks * 0.01150390625;
  }

  // Média das posições esquerda/direita
  public double getAvaregePosition() {
    return (ticksToCentimeters(getLeftPosition()) + ticksToCentimeters(getRightPosition())) / 2;
  }
  // PID automático
  public void autoMove(double setpoint) {
    pidController.setSetpoint(setpoint);

    // Calcula velocidade a partir do PID
    double speed = pidController.calculate(getAvaregePosition());

    // Drive em linha reta
    drive(-speed, 0);

    // Envia ao dashboard
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("SetPoint", setpoint);
  }

  public void drive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);

    // Conversão para tensão (simulação apenas)
    leftVoltage = (speed + rotation) * 6;
    rightVoltage = (speed - rotation) * 6;
  }

  // Para os motores
  public void stop() {
    diffDrive.stopMotor();
    leftVoltage = 0;
    rightVoltage = 0;
  }

  // Simulação

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(leftVoltage, rightVoltage);
    m_driveSim.update(0.02);

    m_field.setRobotPose(m_driveSim.getPose());
  }

  // telemetria
  
  @Override
  public void periodic() {

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

  public void resetPosition() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

}