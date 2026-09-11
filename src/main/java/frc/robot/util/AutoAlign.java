package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import lombok.Getter;

public class AutoAlign {
  private AutoAlign() {
    /* This utility class should not be instantiated */
  }

  @Getter private static Pose2d lastTarget = Pose2d.kZero;

  public static Pose2d getTargetPose() {
    // lowkey not worthy of a whole method but whatever
    lastTarget =
        RobotUtil.isRedAlliance()
            ? Constants.FieldConstants.RED_PANTRY
            : Constants.FieldConstants.BLUE_PANTRY;
    return lastTarget;
  }
}
