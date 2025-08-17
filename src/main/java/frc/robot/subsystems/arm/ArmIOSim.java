package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private DCMotorSim pivotMotorSim;
  private DCMotor pivotMotor = DCMotor.getNEO(1);
  private DCMotorSim intakeMotorSim;
  private DCMotor intakeMotor = DCMotor.getNEO(1);

  public ArmIOSim() {
    pivotMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotMotor, 0.025, 1.0),
            pivotMotor); // TODO: work out what the gearing actually is
    intakeMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(intakeMotor, 0.025, 1.0),
            intakeMotor); // TODO: work out what the gearing actually is
  }

  private double pivotVoltage = 0.0;
  private double intakeVoltage = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    pivotMotorSim.update(0.02);
    intakeMotorSim.update(0.02);

    inputs.pivotAppliedVoltage = pivotVoltage;
    inputs.pivotCurrentAmps = pivotMotorSim.getCurrentDrawAmps();
    inputs.pivotPositionRotations = pivotMotorSim.getAngularPositionRad();
    inputs.pivotAngularVelocityRotationsPerMinute = pivotMotorSim.getAngularVelocityRPM();

    inputs.intakeAppliedVoltage = intakeVoltage;
    inputs.intakeCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.intakePositionRotations = intakeMotorSim.getAngularPositionRad();
    inputs.intakeAngularVelocityRotationsPerMinute = intakeMotorSim.getAngularVelocityRPM();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    this.pivotVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    pivotMotorSim.setInputVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    this.intakeVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    intakeMotorSim.setInputVoltage(voltage);
  }
}
