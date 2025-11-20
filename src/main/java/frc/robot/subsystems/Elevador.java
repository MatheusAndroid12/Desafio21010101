package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Elevador extends SubsystemBase {

    private static final int LEAD_ID = 6;
    private static final int FOLLOWER_ID = 7;

    private final SparkMax elevatorMaster = new SparkMax(LEAD_ID, MotorType.kBrushless);
    private final SparkMax elevatorSlave = new SparkMax(FOLLOWER_ID, MotorType.kBrushless);

    private final RelativeEncoder masterEncoder;
    private final RelativeEncoder slaveEncoder;

    public final PIDController pidControllerElevador = new PIDController(0.05, 0, 0);

    private double target = 0;
    private double maxspeed = 0.3;
    private double speed = 0.3;

    public Elevador() {
        pidControllerElevador.setTolerance(0.2);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        elevatorMaster.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        elevatorSlave.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        masterEncoder = elevatorMaster.getEncoder();
        slaveEncoder = elevatorSlave.getEncoder();

        SmartDashboard.putNumber("Target", target);
        SmartDashboard.putNumber("Velocidade Máxima Elevador", maxspeed);
        SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> {
            masterEncoder.setPosition(0);
            slaveEncoder.setPosition(0);

            var motorMasterconfig = new SparkMaxConfig();

            motorMasterconfig.inverted(false);
            motorMasterconfig.idleMode(IdleMode.kBrake);
            motorMasterconfig.disableFollowerMode();
        
            elevatorMaster.configure(motorMasterconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            
            var slaveMotorconfig = new SparkMaxConfig();
        
            slaveMotorconfig.idleMode(IdleMode.kBrake);
            slaveMotorconfig.disableFollowerMode();
            slaveMotorconfig.inverted(false);
            
            elevatorSlave.configure(slaveMotorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }));
    }

    public void run(double speed){
        elevatorMaster.set(speed);
        elevatorSlave.set(-speed);
      }

    public void levantagem(double controlePos) {
        elevatorMaster.set(speed);
        elevatorSlave.set(-speed);
    }

    public void elevatorPIDMove(double target) {
        pidControllerElevador.setSetpoint(target);
    
        double pidOutput = pidControllerElevador.calculate(getHeight());
        speed = MathUtil.clamp(pidOutput, -maxspeed, maxspeed);
    
        speed = MathUtil.clamp(speed, -0.4, maxspeed);
    
        levantagem(speed);
    }
    

    public void setTarget(double target) {
        this.target = target;
    }

    public double getTarget() {
        return target;
    }

    public double getHeight() {
        return (ticksToMeters(masterEncoder.getPosition()) + -ticksToMeters(slaveEncoder.getPosition())) / 2;
    }

    public boolean inPosition() {
        return pidControllerElevador.atSetpoint();
    }

    private double ticksToMeters(double ticks) {
        return ticks * 0.50107266;
    }

    @Override
    public void periodic() {
        maxspeed = SmartDashboard.getNumber("Velocidade Máxima Elevador", 0.3);
        elevatorPIDMove(target);
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putBoolean("in the setpoint", pidControllerElevador.atSetpoint());
SmartDashboard.putNumber("elevator setpoint", pidControllerElevador.getSetpoint());
        SmartDashboard.putNumber("LeftPower", elevatorMaster.get());
        SmartDashboard.putNumber("RightPower", -elevatorSlave.get());
    }
}