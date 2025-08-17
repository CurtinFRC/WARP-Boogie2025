package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final double pivotGearing =
      1.0 / 1.0; // TODO: work out what the gearing actually is
  public static final double intakeGearing =
      1.0 / 1.0; // TODO: work out what the gearing actually is

  public static final double pivotStowPositionRotations = 5; // TODO: find actual position
  public static final double pivotIntakePositionRotations = 19; // TODO: find actual position
  public static final double pivotEjectPositionRotations = 16; // TODO: find actual position
  public static final double pivotStartPositionRotations = 0.0; // TODO: find actual
  public static final double pivotLollipopIntakePositionRotations = 14.2; // TODO: find actual

  public static final double pivotKp = 0.5; // TODO: find actual value]
  public static final double pivotKi = 0.0; // TODO: find actual value
  public static final double pivotKd = 0.0; // TODO: find actual value
  public static final double pivotToleranceRotations = 0.05; // TODO: find actual value

  public static final double intakeKp = 1.0; // TODO: find actual value
  public static final double intakeKi = 0.0; // TODO: find actual value
  public static final double intakeKd = 0.0; // TODO: find actual value
  public static final double intakeToleranceRPM = 0.05; // TODO: find actual value

  public enum ArmState {
    STOWED,
    INTAKE,
    EJECT,
    START,
    LOLLIPOP_INTAKE,
  }
}
