package frc.robot.util.io.motors.roller;

import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO extends MotorIO {
  @AutoLog
  class RollerIOInputs extends MotorIOInputs {
    public double velocityRPS;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void setVelocity(double rps) {}
}
