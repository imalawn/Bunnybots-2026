// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.LaserCAN;
import frc.robot.util.io.sensors.LaserCANInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final Roller leftRoller;
  private final Roller rightRoller;
  private final LaserCAN beambreak;
  private final LaserCANInputsAutoLogged beambreakInputs = new LaserCANInputsAutoLogged();

  public Outtake() {
    RollerIO leftIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.OUTTAKE_LEFT,
                  OuttakeConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX44(1),
              new MotorIO.RotationalMechanismConstraints(
                  1, OuttakeConstants.OUTTAKE_MOI, 0, 0, 0, 0),
              OuttakeConstants.OUTTAKE_KP,
              OuttakeConstants.OUTTAKE_KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    RollerIO rightIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.OUTTAKE_RIGHT,
                  OuttakeConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX44(1),
              new MotorIO.RotationalMechanismConstraints(
                  1, OuttakeConstants.OUTTAKE_MOI, 0, 0, 0, 0),
              OuttakeConstants.OUTTAKE_KP,
              OuttakeConstants.OUTTAKE_KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    leftRoller = new Roller("Outtake/Left", leftIO);
    rightRoller = new Roller("Outtake/Right", rightIO);

    // id is the same but on different bus
    beambreak = new LaserCAN(Constants.CANConstants.OUTTAKE_LASERCAN);
  }

  @Override
  public void periodic() {
    leftRoller.periodic();
    rightRoller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Outtake/DistanceSensor", beambreakInputs);
  }

  private void runTogether(double rps) {
    leftRoller.runVelocity(rps);
    rightRoller.runVelocity(rps);
  }

  private void stop() {
    leftRoller.stop();
    rightRoller.stop();
  }

  public Command sterilize() {
    return startEnd(
        () -> {
          leftRoller.runVelocity(OuttakeConstants.STERILIZATION_RPS);
          rightRoller.runVelocity(-OuttakeConstants.STERILIZATION_RPS);
        },
        this::stop);
  }

  public Command eject() {
    return startEnd(() -> runTogether(OuttakeConstants.RPS), this::stop);
  }

  public Command reverse() {
    return startEnd(() -> runTogether(OuttakeConstants.REVERSED_RPS), this::stop);
  }

  public boolean hasGamePiece() {
    return beambreakInputs.measurementValid
        && beambreakInputs.distanceMillimeters <= OuttakeConstants.BEAMBREAK_THRESHOLD;
  }
}
