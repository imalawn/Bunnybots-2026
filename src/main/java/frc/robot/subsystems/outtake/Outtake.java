// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.LaserCAN;
import frc.robot.util.io.sensors.LaserCANInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final Roller roller;
  private final LaserCAN beambreak;
  private final LaserCANInputsAutoLogged beambreakInputs = new LaserCANInputsAutoLogged();

  public Outtake() {
    RollerIO io =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.OUTTAKE,
                  OuttakeConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              OuttakeConstants.CONSTRAINTS,
              OuttakeConstants.OUTTAKE_KP,
              OuttakeConstants.OUTTAKE_KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    roller = new Roller("Outtake", io);

    // id is the same but on different bus
    beambreak = new LaserCAN(Constants.CANConstants.OUTTAKE);
  }

  @Override
  public void periodic() {
    roller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Outtake/DistanceSensor", beambreakInputs);
  }

  public Command eject() {
    return startEnd(() -> roller.runVelocity(OuttakeConstants.RPS), roller::stop);
  }

  public Command reverse() {
    return startEnd(() -> roller.runVelocity(OuttakeConstants.REVERSED_RPS), roller::stop);
  }

  public boolean hasGamePiece() {
    return beambreakInputs.measurementValid
        && beambreakInputs.distanceMillimeters <= OuttakeConstants.BEAMBREAK_THRESHOLD;
  }
}
