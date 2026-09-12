package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SimulationHelper {
  public static final Translation3d ELEVATOR_TO_CARROT = new Translation3d(/* TODO find this */ );

  @Getter private static SimulationHelper instance;

  public static SimulationHelper createInstance(
      Elevator elevator,
      Intake intake,
      Indexer indexer,
      Outtake outtake,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    instance =
        new SimulationHelper(elevator, intake, indexer, outtake, driveSimulation, chassisSpeeds);
    return instance;
  }

  private final Elevator elevator;
  private final Intake intake;
  private final Indexer indexer;
  private final Outtake outtake;
  private final SwerveDriveSimulation driveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;
  private final IntakeSimulation intakeSimulation;

  @Getter private boolean outtakeLoaded = false;

  private SimulationHelper(
      Elevator elevator,
      Intake intake,
      Indexer indexer,
      Outtake outtake,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.intake = intake;
    this.indexer = indexer;
    this.outtake = outtake;
    this.driveSimulation = driveSimulation;
    this.chassisSpeeds = chassisSpeeds;

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Carrot",
            driveSimulation,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.BACK,
            3);
  }

  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] carrotPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Carrot");

    Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);
    // to set up the model
    Logger.recordOutput("FieldSimulation/Carrots", carrotPoses);

    if (!outtakeLoaded
        && getNumHeldCarrots() > 0
        && indexer.getVelocityRPS() > 50
        && elevator.getSetpoint() == Elevator.Setpoint.STOWED) {
      outtakeLoaded = true;
      Logger.recordOutput("FieldSimulation/IsOuttakeLoaded", outtakeLoaded);
    }

    Logger.recordOutput(
        "FieldSimulation/RobotComponentPositions",
        // elevator
        new Pose3d(0.0, 0.0, elevator.getPositionMeters(), Rotation3d.kZero),
        // intake
        new Pose3d(
            -0.305,
            0,
            0.23,
            new Rotation3d(0, Math.toRadians(42.5 - intake.getPivotPosition()), 0)));
  }

  /** Returns the total number of carrots in the robot. */
  @AutoLogOutput(key = "FieldSimulation/NumCarrotsInBot")
  public int getNumHeldCarrots() {
    return intakeSimulation.getGamePiecesAmount();
  }

  /** Returns the number of carrots in the robot's hopper (excludes carrot in outtake). */
  public int getNumCarrotsInHopper() {
    return outtakeLoaded ? getNumHeldCarrots() - 1 : getNumHeldCarrots();
  }

  /** Returns whether there is a game piece in the robot at all */
  public boolean hasAnyGamePiece() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  public void score() {
    if (!outtakeLoaded) {
      return;
    }

    intakeSimulation.obtainGamePieceFromIntake();
    outtakeLoaded = false;

    Pose3d globalPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevator.getPositionMeters());
    Translation3d carrotTranslation =
        globalPose.getTranslation().plus(elevatorTranslation).plus(ELEVATOR_TO_CARROT);

    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(carrotTranslation.getX(), carrotTranslation.getY()),
            chassisSpeeds.get(),
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(carrotTranslation.getZ()),
            MetersPerSecond.of(1),
            Degrees.of(0 /* TODO put outtake angle here (static angle) */));

    carrotOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }

  public void dropCarrot(HarvestHavocCarrotOnFly.CarrotStations side) {
    HarvestHavocCarrotOnFly coralOnFly =
        HarvestHavocCarrotOnFly.dropFromCarrotStation(
            side, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
}
