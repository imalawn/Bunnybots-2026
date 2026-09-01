package frc.robot.util.subsystems;

import java.util.HashSet;
import java.util.Set;
import lombok.Getter;

public class RobotStateHandler {
  private static RobotStateHandler instance;
  @Getter private static boolean robotEnabled;

  public static synchronized RobotStateHandler getInstance() {
    if (instance == null) {
      instance = new RobotStateHandler();
    }
    return instance;
  }

  private final Set<ExtendedSubsystem> subsystems = new HashSet<>();

  public void registerSubsystem(ExtendedSubsystem... subsystems) {
    for (ExtendedSubsystem subsystem : subsystems) {
      if (subsystem == null) {
        continue;
      }
      if (this.subsystems.contains(subsystem)) {
        continue;
      }
      this.subsystems.add(subsystem);
    }
  }

  public void disable() {
    robotEnabled = false;
    subsystems.forEach(ExtendedSubsystem::disable);
  }

  public void enable() {
    robotEnabled = true;
    subsystems.forEach(ExtendedSubsystem::enable);
  }
}
