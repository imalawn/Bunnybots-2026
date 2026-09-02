package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.SETPOINTS;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.pivot.Pivot;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.pivot.PivotIOSim;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.EncoderIO;
import frc.robot.util.io.sensors.EncoderIOCANcoder;
import frc.robot.util.subsystems.ExtendedSubsystem;
import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends ExtendedSubsystem {
  public enum PivotState {
    STOWED,
    GROUND,
    SOURCE,
    HANDOFF
  }

  private final Roller roller;
  private final Pivot pivot;

  @Getter private PivotState pivotState = PivotState.STOWED;

  public Intake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.INTAKE_PIVOT,
                  IntakeConstants.PIVOT_CONFIG)
              .addControlRequest(new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true))
              .addControlRequest(new PositionVoltage(0).withOverrideBrakeDurNeutral(true))
              .build();
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.PIVOT_GEAR_RATIO,
                  SingleJointedArmSim.estimateMOI(0.5, 2),
                  0.5,
                  0,
                  SETPOINTS.get(PivotState.GROUND).in(Radians),
                  0),
              IntakeConstants.PIVOT_KP,
              IntakeConstants.PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.INTAKE_ROLLER,
                  IntakeConstants.ROLLER_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_MOI, 0.2, 0, 0, 0),
              IntakeConstants.ROLLER_KP * 20,
              IntakeConstants.ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };
    EncoderIO encoderIO =
        switch (Constants.currentMode) {
            // todo figure out how CANCoder simulation works
          case REAL, SIM -> new EncoderIOCANcoder(
              Constants.CANConstants.SUPERSTRUCTURE,
              Constants.CANConstants.INTAKE_ENCODER,
              IntakeConstants.ENCODER_CONFIG);
          default -> inputs -> {};
        };

    pivot = new Pivot("Intake/Pivot", pivotIO, encoderIO);
    roller = new Roller("Intake/Roller", rollerIO);

    Logger.recordOutput("Intake/PivotState", pivotState.toString());
  }

  @Override
  public void disable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void enable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void periodic() {
    pivot.periodic();
    roller.periodic();
  }

  private void runSetpoint(PivotState newState) {
    pivot.runPosition(SETPOINTS.get(newState));
    pivotState = newState;
    Logger.recordOutput("Intake/State", pivotState.toString());
  }

  public Command intakeFromGround() {
    return startEnd(
        () -> {
          runSetpoint(PivotState.GROUND);
          roller.runVelocity(IntakeConstants.ROLLER_RPS);
        },
        roller::stop);
  }

  public Command handoff() {
    AtomicReference<PivotState> previous = new AtomicReference<>();
    return startEnd(
        () -> {
          previous.set(pivotState);
          runSetpoint(PivotState.HANDOFF);
          roller.runVelocity(IntakeConstants.ROLLER_RPS);
        },
        () -> {
          roller.stop();
          runSetpoint(previous.get());
        });
  }

  public Command stow() {
    return runOnce(
        () -> {
          runSetpoint(PivotState.STOWED);
          roller.stop();
        });
  }

  public double getPivotPosition() {
    return pivot.getPositionDeg();
  }

  public double getRollerRPS() {
    return roller.getVelocityRPS();
  }
}
