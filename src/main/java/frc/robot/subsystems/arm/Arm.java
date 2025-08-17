package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final PIDController pivotController =
      new PIDController(ArmConstants.pivotKp, ArmConstants.pivotKi, ArmConstants.pivotKd, 0.02);
  private final PIDController intakeController =
      new PIDController(ArmConstants.intakeKp, ArmConstants.intakeKi, ArmConstants.intakeKd, 0.02);

  public Arm(ArmIO io) {
    this.io = io;
    pivotController.setTolerance(ArmConstants.pivotToleranceRotations);
    intakeController.setTolerance(ArmConstants.intakeToleranceRPM);
    setDefaultCommand(run(() -> stop()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void pivotToSetpoint(ArmState state) {
    double out =
        pivotController.calculate(
            inputs.pivotPositionRotations,
            (switch (state) {
              case STOWED:
                yield ArmConstants.pivotStowPositionRotations;
              case INTAKE:
                yield ArmConstants.pivotIntakePositionRotations;
              case START:
                yield ArmConstants.pivotStartPositionRotations;
              case EJECT:
                yield ArmConstants.pivotEjectPositionRotations;
            }));
    Logger.recordOutput("Arm/pivotAppliedVoltage", out);
    Logger.recordOutput("Arm/pivotError", pivotController.getError());
    Logger.recordOutput("Pivot/TargetState", state.name());
    Logger.recordOutput("Arm/PivotAtSetpoint", pivotController.atSetpoint());
    io.setPivotVoltage(out);
  }
  ;

  // public Command intakeToSpeed(double adjustment) {
  //   return run(
  //     () -> {
  //       double out =
  //         intakeController.calculate(
  //           inputs.intakeAngularVelocityRotationsPerMinute,
  //           intake
  //         )
  //     }
  //   )
  // }

  public Command intake() {
    return run(
        () -> {
          pivotToSetpoint(ArmState.INTAKE);
          intakeRaw(-12);
        });
  }

  public Command hold() {
    return run(
        () -> {
          pivotToSetpoint(ArmState.STOWED);
          intakeRaw(-0.5);
        });
  }

  public Command ejectPrep() {
    return run(
        () -> {
          pivotToSetpoint(ArmState.EJECT);
          intakeRaw(0);
        });
  }

  public Command eject() {
    return run(
        () -> {
          pivotToSetpoint(ArmState.EJECT);
          intakeRaw(12);
        });
  }

  public void intakeRaw(double voltage) {
    io.setIntakeVoltage(voltage);
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
