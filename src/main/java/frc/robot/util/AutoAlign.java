package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
  private AutoAlign() {
    /* This utility class should not be instantiated */
  }

  @Getter private static Pose2d lastTarget = Pose2d.kZero;

  public static Pose2d getTargetPose() {
    lastTarget =
        RobotUtil.isRedAlliance()
            ? Constants.FieldConstants.RED_RAMP
            : Constants.FieldConstants.BLUE_RAMP;
    Logger.recordOutput("AutoAlign/TargetPose", lastTarget);
    return lastTarget;
  }
}
