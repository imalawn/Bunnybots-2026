// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.LaserCanIO;
import frc.robot.util.io.sensors.LaserCanIOInputsAutoLogged;
import frc.robot.util.io.sensors.LaserCanIOReal;
import frc.robot.util.sim.SimulationHelper;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final Roller roller;
  private final LaserCanIO beambreak;
  private final LaserCanIOInputsAutoLogged beambreakInputs = new LaserCanIOInputsAutoLogged();

  public Indexer() {
    RollerIO io =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.INDEXER,
                  IndexerConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  1, IndexerConstants.INDEXER_MOI, 0, 0, 0, 0),
              IndexerConstants.INDEXER_KP,
              IndexerConstants.INDEXER_KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    roller = new Roller("Indexer", io);

    beambreak =
        switch (Constants.currentMode) {
          case REAL -> new LaserCanIOReal(Constants.CANConstants.INDEXER_LASERCAN);
          case SIM -> LaserCanIO.beambreakSim(
              () -> SimulationHelper.getInstance().getNumCarrotsInHopper() > 0,
              IndexerConstants.BEAMBREAK_THRESHOLD);
          case REPLAY -> inputs -> {};
        };
  }

  @Override
  public void periodic() {
    roller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Indexer/DistanceSensor", beambreakInputs);
  }

  public Command feed() {
    return startEnd(() -> roller.runVelocity(IndexerConstants.RPS), roller::stop);
  }

  public Command reverse() {
    return startEnd(() -> roller.runVelocity(IndexerConstants.REVERSED_RPS), roller::stop);
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public boolean hasGamePiece() {
    return beambreakInputs.measurementValid
        && beambreakInputs.distanceMillimeters <= IndexerConstants.BEAMBREAK_THRESHOLD;
  }
}
