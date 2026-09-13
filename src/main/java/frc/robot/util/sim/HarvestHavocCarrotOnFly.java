package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class HarvestHavocCarrotOnFly extends GamePieceProjectile {
  public HarvestHavocCarrotOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTouchGroundHeight(0.2);
  }

  public HarvestHavocCarrotOnFly(
      Translation2d initialPosition,
      Translation2d initialLaunchingVelocityMPS,
      double initialHeight,
      double initialVerticalSpeedMPS,
      Rotation3d gamePieceRotation) {
    super(
        HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
        initialPosition,
        initialLaunchingVelocityMPS,
        initialHeight,
        initialVerticalSpeedMPS,
        gamePieceRotation);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTouchGroundHeight(0.2);
  }

  public enum CarrotStations {
    BLUE_RAMP(
        new Pose2d(FieldConstants.BLUE_RAMP.getTranslation(), Rotation2d.kCW_90deg),
        Centimeters.of(57.659358)),
    RED_RAMP(
        new Pose2d(FieldConstants.RED_RAMP.getTranslation(), Rotation2d.kCCW_90deg),
        Centimeters.of(57.659358)),
    BLUE_REAR_DEPOT(FieldConstants.BLUE_REAR_DEPOT, Centimeters.of(57.383772)),
    BLUE_SIDE_DEPOT(FieldConstants.BLUE_SIDE_DEPOT, Centimeters.of(57.383772)),
    RED_REAR_DEPOT(FieldConstants.RED_REAR_DEPOT, Centimeters.of(57.383772)),
    RED_SIDE_DEPOT(FieldConstants.RED_SIDE_DEPOT, Centimeters.of(57.383772));

    private final Pose2d startingPose;
    private final Distance height;

    CarrotStations(Pose2d startingPose, Distance height) {
      this.startingPose = startingPose;
      this.height = height;
    }
  }

  public static HarvestHavocCarrotOnFly dropFromCarrotStation(
      CarrotStations station, DriverStation.Alliance alliance) {
    return new HarvestHavocCarrotOnFly(
        station.startingPose.getTranslation(),
        new Translation2d(alliance == DriverStation.Alliance.Red ? -1.816 : 1.816, 0),
        station.height.in(Meters),
        -0.4,
        new Rotation3d(station.startingPose.getRotation()));
  }

  @Override
  public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
    if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
    simulatedArena.addGamePiece(
        new GamePieceOnFieldSimulation(
            HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
            () ->
                Math.max(
                    HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO.gamePieceHeight().in(Meters)
                        / 2,
                    getPositionAtTime(super.launchedTimer.get()).getZ()),
            new Pose2d(
                getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                gamePieceRotation.toRotation2d()),
            super.initialLaunchingVelocityMPS));
  }
}
