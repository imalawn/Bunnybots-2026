package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import java.util.Set;
import lombok.Getter;

public class AutoAlign {
  private AutoAlign() {
    /* This utility class should not be instantiated */
  }

  private static final Set<Pose2d> RED_PANTRIES =
      Set.of(Constants.FieldConstants.RED_LEFT_PANTRY, Constants.FieldConstants.RED_RIGHT_PANTRY);
  private static final Set<Pose2d> BLUE_PANTRIES =
      Set.of(Constants.FieldConstants.BLUE_LEFT_PANTRY, Constants.FieldConstants.BLUE_RIGHT_PANTRY);

  @Getter private static Pose2d lastTarget = Pose2d.kZero;

  public static Pose2d getTarget(Pose2d robotPose) {
    lastTarget = robotPose.nearest(RobotUtil.isRedAlliance() ? RED_PANTRIES : BLUE_PANTRIES);
    return lastTarget;
  }
}
