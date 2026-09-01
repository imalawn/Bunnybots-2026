package frc.robot.commands;

public class Autos {
  // system check
  private static final double DRIVING_SPEED = 2.5;
  private static final double WAIT_SHORT = 3.0;
  private static final double WAIT_LONG = 5.0;

  // auto cycle
  private static final double INTAKE_TIME = 5.0;
  private static final double AGITATE_START = 3.0;
  private static final double SHOOT_TIME = 5.0;

  //  public static Command systemCheck(Drive drive, Shooter shooter, Feeder feeder, Intake intake)
  // {
  //    return Commands.sequence(
  //            Commands.runOnce(() -> Logger.recordOutput("Autos/SystemCheck/State", "Drive")),
  //            // forwards
  //            Commands.run(
  //                    () -> drive.runVelocity(new ChassisSpeeds(DRIVING_SPEED, 0, 0), true),
  // drive)
  //                .withTimeout(WAIT_SHORT),
  //            // right
  //            Commands.run(
  //                    () -> drive.runVelocity(new ChassisSpeeds(0, -DRIVING_SPEED, 0), true),
  // drive)
  //                .withTimeout(WAIT_SHORT),
  //            // spin clockwise
  //            Commands.run(
  //                    () -> drive.runVelocity(new ChassisSpeeds(0.0, 0, -DRIVING_SPEED), true),
  // drive)
  //                .withTimeout(WAIT_SHORT),
  //            Commands.runOnce(drive::stop, drive),
  //            Commands.runOnce(() -> Logger.recordOutput("Autos/SystemCheck/State", "Intake")),
  //            intake.intake().withTimeout(WAIT_LONG),
  //            Commands.runOnce(() -> ShooterConstants.DEFAULT_RPM.set(1500)),
  //            Commands.runOnce(
  //                () -> Logger.recordOutput("Autos/SystemCheck/State", "AgitateAndShoot")),
  //            Commands.parallel(intake.agitate(feeder), shooter.shootDefault(feeder))
  //                .withTimeout(WAIT_LONG))
  //        .finallyDo(() -> Logger.recordOutput("Autos/SystemCheck/State", "Idle"));
  //  }
  //
  //  public static Command leftCycle(
  //      Drive drive, Vision vision, Shooter shooter, Feeder feeder, Intake intake) {
  //    Pose2d preIntake =
  //        new Pose2d(isRedAlliance() ? RED_LEFT_MID : BLUE_LEFT_MID, Rotation2d.kCW_Pi_2);
  //    Pose2d scoring =
  //        new Pose2d(
  //            isRedAlliance() ? RED_LEFT_SCORING : BLUE_LEFT_SCORING,
  // Rotation2d.fromDegrees(45.0));
  //    return cycle(drive, vision, shooter, feeder, intake, preIntake, scoring);
  //  }
  //
  //  public static Command rightCycle(
  //      Drive drive, Vision vision, Shooter shooter, Feeder feeder, Intake intake) {
  //    Pose2d preIntake =
  //        new Pose2d(isRedAlliance() ? RED_RIGHT_MID : BLUE_RIGHT_MID, Rotation2d.kCCW_Pi_2);
  //    Pose2d scoring =
  //        new Pose2d(
  //            isRedAlliance() ? RED_RIGHT_SCORING : BLUE_RIGHT_SCORING,
  //            Rotation2d.fromDegrees(-45.0));
  //    return cycle(drive, vision, shooter, feeder, intake, preIntake, scoring);
  //  }
  //
  //  private static Command cycle(
  //      Drive drive,
  //      Vision vision,
  //      Shooter shooter,
  //      Feeder feeder,
  //      Intake intake,
  //      Pose2d preIntake,
  //      Pose2d scoring) {
  //    Command autoAlign = NamedCommands.getCommand("Auto Align");
  //
  //    return Commands.repeatingSequence(
  //        intake.deploy(),
  //        drive.driveToPose(preIntake),
  //        drive
  //            .driveToPose(vision::getObjectPose) // vision method not impl yet
  //            .withTimeout(INTAKE_TIME),
  //        intake.runOnce(intake::stop),
  //        drive.driveToPose(scoring),
  //        autoAlign,
  //        shooter
  //            .shoot(feeder)
  //            .alongWith(Commands.waitSeconds(AGITATE_START).andThen(intake.agitate(feeder)))
  //            .withTimeout(SHOOT_TIME));
  //  }
}
