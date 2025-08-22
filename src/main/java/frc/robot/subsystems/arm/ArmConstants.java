package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final double pivotGearing = 1.0 / 1.0;
  public static final double intakeGearing = 1.0 / 1.0;

  public static final double pivotStowPositionRotations = 5;
  public static final double pivotIntakePositionRotations = 21;
  public static final double pivotEjectPositionRotations = 16;
  public static final double pivotStartPositionRotations = 0.0;

  public static final double pivotKp = 0.5;
  public static final double pivotKi = 0.0;
  public static final double pivotKd = 0.0;
  public static final double pivotToleranceRotations = 0.05;

  public static final double intakeHoldVoltage = 2;

  public enum ArmState {
    STOWED,
    INTAKE,
    EJECT,
    START,
  }
}
