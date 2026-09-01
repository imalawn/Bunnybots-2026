package frc.robot.util.io.motors.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller extends Motor<RollerIO, RollerIOInputsAutoLogged> {
  public Roller(String name, RollerIO io, BooleanSupplier brakeMode) {
    super(name, io, new RollerIOInputsAutoLogged(), in -> {}, brakeMode);
    io.configure(false, true);
  }

  public Roller(String name, RollerIO io) {
    this(name, io, () -> false);
  }

  /** {@inheritDoc} */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runVelocity(double rps) {
    if (tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRPS);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }
}
