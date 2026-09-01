package frc.robot.util.io.motors.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.io.motors.MotorIOSim;

public class RollerIOSim extends MotorIOSim implements RollerIO {
  private final FlywheelSim sim;

  public RollerIOSim(
      DCMotor motorModel,
      RotationalMechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                motorModel, constraints.moi(), constraints.reduction()),
            motorModel);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (isClosedLoop) {
      appliedVoltage =
          MathUtil.clamp(
              pid.calculate(Units.radiansToRotations(sim.getAngularVelocityRadPerSec())),
              -12.0,
              12.0);
    }
    updateMotorInputs(inputs);
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);
    inputs.velocityRPS = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double rps) {
    pid.setSetpoint(rps);
    isClosedLoop = true;
  }
}
