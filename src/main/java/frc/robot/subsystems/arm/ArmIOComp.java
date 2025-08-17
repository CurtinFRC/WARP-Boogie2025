package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.SparkUtil;

public class ArmIOComp implements ArmIO {
  private final SparkMax pivotMotor = new SparkMax(4, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final SparkMax intakeMotor = new SparkMax(3, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  public ArmIOComp() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(0, 60).idleMode(IdleMode.kBrake).inverted(false).openLoopRampRate(1.0);

    SparkUtil.tryUntilOk(
        5,
        () ->
            pivotMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        5,
        () ->
            intakeMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.pivotAppliedVoltage = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.pivotPositionRotations = pivotEncoder.getPosition();
    inputs.pivotAngularVelocityRotationsPerMinute = pivotEncoder.getVelocity();

    inputs.intakeAppliedVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakePositionRotations = intakeEncoder.getPosition();
    inputs.intakeAngularVelocityRotationsPerMinute = intakeEncoder.getVelocity();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    final double pivotVoltage = MathUtil.clamp(voltage, -4.0, 4.0);
    pivotMotor.setVoltage(pivotVoltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    final double intakeVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    intakeMotor.setVoltage(intakeVoltage);
  }
}
