package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double pivotAppliedVoltage;
    public double pivotCurrentAmps;
    public double pivotPositionRotations;
    public double pivotAngularVelocityRotationsPerMinute;

    public double intakeAppliedVoltage;
    public double intakeCurrentAmps;
    public double intakePositionRotations;
    public double intakeAngularVelocityRotationsPerMinute;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void setPivotVoltage(double voltage) {}

  public default void pivotToPositionRotations(ArmConstants.ArmState state) {}

  public default void intakeToSpeed(double speed) {}
}
