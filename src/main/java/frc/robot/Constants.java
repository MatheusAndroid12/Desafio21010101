package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final double kElevatorkS = 0.0; 
  public static final double kElevatorkG = 0.762; 
  public static final double kElevatorkV = 0.762; 
  public static final double kElevatorkA = 0.0;

  public static final double kElevatorGearing = 10.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0); 
  public static final double kCarriageMass = 4.0;

  public static final double kSetpointMeters = 0.75; 
  public static final double kMinElevatorHeightMeters = 0.0; 
  public static final double kMaxElevatorHeightMeters = 1.25; 
  public static final int kJoystickPort = 0;
  public static class OperatorConstants {
    }
}