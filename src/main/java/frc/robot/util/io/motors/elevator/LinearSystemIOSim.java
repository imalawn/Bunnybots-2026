package frc.robot.util.io.motors.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.io.motors.MotorIOSim;

import static edu.wpi.first.units.Units.Radians;

public class LinearSystemIOSim extends MotorIOSim implements LinearSystemIO {
  private final ElevatorSim sim;

  private final double gearing;
  private final double drumRadiusMeters;

  private double targetPositionRad = 0.0;

  public LinearSystemIOSim(
      DCMotor gearbox,
      LinearMechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);

    this.gearing = constraints.reduction();
    this.drumRadiusMeters = constraints.drumRadiusMeters();

    this.sim =
        new ElevatorSim(
            gearbox,
            gearing,
            constraints.carriageMassKg(),
            drumRadiusMeters,
            constraints.minHeightMeters(),
            constraints.maxHeightMeters(),
            true,
            0.0);
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    if (isClosedLoop) {
      double currentMotorRad = carriageMetersToRad(sim.getPositionMeters());
      appliedVoltage = MathUtil.clamp(pid.calculate(currentMotorRad, targetPositionRad), -12, 12);
    }
    updateMotorInputs(inputs);

    sim.setInputVoltage(appliedVoltage);
    sim.update(0.020);
    inputs.positionRad = carriageMetersToRad(sim.getPositionMeters());
    inputs.velocityRadPerSec = carriageMetersToRad(sim.getVelocityMetersPerSecond());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(Angle angle) {
    this.targetPositionRad = angle.in(Radians);
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(Angle angle) {
    sim.setState(radsToCarriageMeters(angle.in(Radians)), 0.0);
  }

  private double carriageMetersToRad(double meters) {
    double drumRotations = meters / drumRadiusMeters;
    return drumRotations * gearing;
  }

  private double radsToCarriageMeters(double radians) {
    double drumRadians = radians / gearing;
    return drumRadians * drumRadiusMeters;
  }
}
