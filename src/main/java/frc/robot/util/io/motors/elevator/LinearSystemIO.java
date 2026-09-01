package frc.robot.util.io.motors.elevator;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface LinearSystemIO extends MotorIO {
  @AutoLog
  class LinearSystemIOInputs extends MotorIOInputs {
    public double positionRad;
    public double velocityRadPerSec;
  }

  default void updateInputs(LinearSystemIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setVelocity(double rps) {}

  default void resetPosition(Angle angle) {}
}
