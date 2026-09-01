package frc.robot.util.io.motors.pivot;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO extends MotorIO {
  @AutoLog
  class PivotIOInputs extends MotorIOInputs {
    public double positionDeg;
    public double velocityDegPerSec;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setPosition(int slot, Angle angle) {
    setPosition(angle);
  }

  default void resetPosition(Angle angle) {}
}
