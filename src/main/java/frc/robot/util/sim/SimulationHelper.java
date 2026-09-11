package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulationHelper {
  @Getter private static SimulationHelper instance;

  public static SimulationHelper createInstance(
      Elevator elevator,
      Intake intake,
      Outtake outtake,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    instance = new SimulationHelper(elevator, intake, outtake, driveSimulation, chassisSpeeds);
    return instance;
  }

  private final Elevator elevator;
  private final Intake intake;
  private final Outtake outtake;
  private final SwerveDriveSimulation driveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final IntakeSimulation intakeSimulation;

  private Translation3d outtakeTranslation;
  private Rotation3d outtakeRotation;
  private Pose3d localCoral;

  private SimulationHelper(
      Elevator elevator,
      Intake intake,
      Outtake outtake,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.intake = intake;
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
  }

  private Pose3d getCoralRobotRelativePose() {
    return Pose3d.kZero;
  }

  public boolean isLoaded() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  public void score() {
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return;
    }
  }

  public void loadFuel(HarvestHavocCarrotOnFly.CarrotStations side) {
    HarvestHavocCarrotOnFly coralOnFly =
        HarvestHavocCarrotOnFly.dropFromCarrotStation(
            side, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
}
