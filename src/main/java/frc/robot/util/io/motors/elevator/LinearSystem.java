package frc.robot.util.io.motors.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.sensors.EncoderIO;
import frc.robot.util.subsystems.SubsystemManager;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public class LinearSystem extends Motor<LinearSystemIO, LinearSystemIOInputsAutoLogged> {
  private final Function<Distance, Angle> distanceToAngle;

  private LinearSystem(
      String name,
      LinearSystemIO io,
      EncoderIO encoderIO,
      BooleanSupplier brakeMode,
      Function<Distance, Angle> distanceToAngle) {
    super(name, io, new LinearSystemIOInputsAutoLogged(), encoderIO, brakeMode);
    io.configure(true, true);
    this.distanceToAngle = distanceToAngle;
  }

  /** {@inheritDoc} */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runPosition(Angle angle) {
    if (tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointRad", angle.in(Radians));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runPosition(Distance position) {
    runPosition(distanceToAngle.apply(position));
  }

  public void runVelocity(double rps) {
    if (tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Radians.of(inputs.positionRad);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public static class Builder {
    private final String name;
    private final LinearSystemIO io;
    private EncoderIO encoderIO = inputs -> {};
    private BooleanSupplier brakeMode = SubsystemManager::isRobotEnabled;
    private Function<Distance, Angle> distanceToAngle;

    public Builder(String name, LinearSystemIO io) {
      this.name = name;
      this.io = io;
    }

    public Builder addEncoder(EncoderIO encoderIO) {
      this.encoderIO = encoderIO;
      return this;
    }

    public Builder setBrakeMode(BooleanSupplier brakeMode) {
      this.brakeMode = brakeMode;
      return this;
    }

    public Builder setDrumRadius(double drumRadiusMeters) {
      if (distanceToAngle == null) {
        distanceToAngle = distance -> Radians.of(distance.in(Meters) / drumRadiusMeters);
      }
      return this;
    }

    public Builder useCustomPositionFunction(Function<Distance, Angle> distanceToAngle) {
      this.distanceToAngle = distanceToAngle;
      return this;
    }

    public LinearSystem build() {
      if (distanceToAngle == null) {
        distanceToAngle = distance -> Rotations.zero();
      }
      return new LinearSystem(name, io, encoderIO, brakeMode, distanceToAngle);
    }
  }
}
