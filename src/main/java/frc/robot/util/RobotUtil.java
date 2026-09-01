package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.PriorityQueue;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * This class contains methods that are used throughout the codebase and are not bound to one
 * subsystem or class.
 */
public final class RobotUtil {
  public record RumbleRequest(double left, double right, int priority)
      implements Comparable<RumbleRequest> {
    public RumbleRequest(double intensity, int priority) {
      this(intensity, intensity, priority);
    }

    @Override
    public int compareTo(RumbleRequest other) {
      return other.priority - this.priority;
    }
  }

  @Setter private static CommandXboxController driverController;
  @Setter private static CommandXboxController operatorController;
  private static final PriorityQueue<RumbleRequest> driverRumble = new PriorityQueue<>();
  private static final PriorityQueue<RumbleRequest> operatorRumble = new PriorityQueue<>();

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /** Request driver controller rumble. */
  public static void requestDriverRumble(RumbleRequest request) {
    if (request == null) return;
    if (!driverRumble.contains(request)) {
      driverRumble.offer(request);
      updateDriverRumble();
    }
  }

  /** Stop driver controller rumble. */
  public static void stopDriverRumble(RumbleRequest request) {
    if (request == null) return;
    if (driverRumble.remove(request)) {
      updateDriverRumble();
    }
  }

  /** Request operator controller rumble. */
  public static void requestOperatorRumble(RumbleRequest request) {
    if (request == null) return;
    if (!operatorRumble.contains(request)) {
      operatorRumble.offer(request);
      updateOperatorRumble();
    }
  }

  /** Stop operator controller rumble. */
  public static void stopOperatorRumble(RumbleRequest request) {
    if (request == null) return;
    if (operatorRumble.remove(request)) {
      updateOperatorRumble();
    }
  }

  /** Update driver controller rumbles based on current queue head */
  private static void updateDriverRumble() {
    if (driverController == null) return;
    Logger.recordOutput("XboxController/DriverRumble", driverRumble.toArray(new RumbleRequest[0]));
    RumbleRequest active = driverRumble.peek();
    if (active != null) {
      driverController.setRumble(GenericHID.RumbleType.kLeftRumble, active.left);
      driverController.setRumble(GenericHID.RumbleType.kRightRumble, active.right);
    } else {
      driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
      driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
  }

  /** Update operator controller rumbles based on current queue head */
  private static void updateOperatorRumble() {
    if (operatorController == null) return;
    Logger.recordOutput(
        "XboxController/OperatorRumble", operatorRumble.toArray(new RumbleRequest[0]));
    RumbleRequest active = operatorRumble.peek();
    if (active != null) {
      operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, active.left);
      operatorController.setRumble(GenericHID.RumbleType.kRightRumble, active.right);
    } else {
      operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
      operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
  }
}
