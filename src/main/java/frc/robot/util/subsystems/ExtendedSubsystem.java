package frc.robot.util.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ExtendedSubsystem extends SubsystemBase {
  public ExtendedSubsystem() {
    super();
    RobotStateHandler.getInstance().registerSubsystem(this);
  }

  public void enable() {}

  public void disable() {}
}
