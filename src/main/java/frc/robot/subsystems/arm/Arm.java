package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
    setDefaultCommand(run(() -> stop()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command intakeRaw(double voltage) {
    return run(() -> io.setIntakeVoltage(voltage));
  }

  public Command pivotRaw(double voltage) {
    return run(() -> io.setPivotVoltage(voltage));
  }

  public Command stop() {
    return run(
        () -> {
          io.setIntakeVoltage(0);
          io.setPivotVoltage(0);
        });
  }
}
