package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/**
 * A class with several static methods to help field-related tasks, such as getting the pose of
 * something on the field
 */
public class FieldUtils {
  private static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  public static Pose3d allianceRelativeFlip(Pose3d pose) {
    if (shouldFlip()) {
      return new Pose3d(
          FieldConstants.FIELD_LENGTH.minus(pose.getMeasureX()),
          FieldConstants.FIELD_WIDTH.minus(pose.getMeasureY()),
          pose.getMeasureZ(),
          pose.getRotation().rotateBy(new Rotation3d(Rotation2d.kPi)));
    } else {
      return pose;
    }
  }

  public static Pose2d allianceRelativeFlip(Pose2d pose) {
    return allianceRelativeFlip(new Pose3d(pose)).toPose2d();
  }

  public static Rotation2d allianceRelativeFlip(Rotation2d rotation) {
    if (shouldFlip()) {
      return Rotation2d.k180deg.plus(rotation);
    }
    return rotation;
  }
}
