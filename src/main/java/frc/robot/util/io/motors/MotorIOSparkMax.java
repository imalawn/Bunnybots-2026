package frc.robot.util.io.motors;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.elevator.LinearSystemIO;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.sensors.EncoderIO;

public class MotorIOSparkMax implements RollerIO, PivotIO, LinearSystemIO {
  private static final SparkBaseConfig coastConfig =
      new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
  private static final SparkBaseConfig brakeConfig =
      new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

  private final SparkMax leader;
  private final SparkMax[] followers;

  // the unit used for the motor's conversion factor
  private final AngleUnit positionUnit;
  private final AngularVelocityUnit velocityUnit;

  private final SparkClosedLoopController controller;
  private SparkBase.ControlType positionControlType = SparkBase.ControlType.kPosition;

  private final RelativeEncoder leaderEncoder;

  private boolean brakeMode;

  @SuppressWarnings("resource")
  public MotorIOSparkMax(
      int id,
      SparkBaseConfig config,
      AngleUnit positionUnit,
      AngularVelocityUnit velocityUnit,
      int[] followerIds,
      boolean[] followersOpposed) {
    leader = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    followers = new SparkMax[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new SparkMax(followerIds[i], SparkLowLevel.MotorType.kBrushless);
    }

    tryUntilOk(
        5,
        () ->
            leader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    for (int i = 0; i < followers.length; i++) {
      config.follow(leader.getDeviceId(), followersOpposed[i]);
      SparkMax follower = followers[i];
      tryUntilOk(
          5,
          () ->
              follower.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    this.positionUnit = positionUnit;
    this.velocityUnit = velocityUnit;

    controller = leader.getClosedLoopController();

    leaderEncoder = leader.getEncoder();
    leaderEncoder.setPosition(0);

    this.brakeMode = leader.configAccessor.getIdleMode() == SparkBaseConfig.IdleMode.kBrake;
  }

  public MotorIOSparkMax(
      int id, SparkBaseConfig config, AngleUnit positionUnit, AngularVelocityUnit velocityUnit) {
    this(id, config, positionUnit, velocityUnit, new int[0], new boolean[0]);
  }

  public MotorIOSparkMax withPositionControlType(SparkBase.ControlType controlType) {
    switch (controlType) {
      case kPosition:
      case kMAXMotionPositionControl:
        this.positionControlType = controlType;
        break;
      default:
        throw new IllegalArgumentException("ControlType not supported: " + controlType);
    }
    return this;
  }

  private void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = !leader.hasActiveFault();
    inputs.appliedVoltage = leader.getBusVoltage() * leader.getAppliedOutput();
    inputs.statorCurrentAmps = leader.getOutputCurrent();
    inputs.tempCelsius = leader.getMotorTemperature();

    for (int i = 0; i < followers.length; i++) {
      inputs.followerConnected[i] = !followers[i].hasActiveFault();
      inputs.followerTempCelsius[i] = followers[i].getMotorTemperature();
    }
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRPS = RotationsPerSecond.convertFrom(leaderEncoder.getVelocity(), velocityUnit);
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionDeg = Degrees.convertFrom(leaderEncoder.getPosition(), positionUnit);
    inputs.velocityDegPerSec =
        DegreesPerSecond.convertFrom(leaderEncoder.getVelocity(), velocityUnit);
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    inputs.positionRad = Radians.convertFrom(leaderEncoder.getPosition(), positionUnit);
    inputs.velocityRadPerSec =
        RadiansPerSecond.convertFrom(leaderEncoder.getVelocity(), velocityUnit);
    updateMotorInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPosition(Angle angle) {
    controller.setSetpoint(angle.in(positionUnit), positionControlType);
  }

  @Override
  public void setVelocity(double rps) {
    controller.setSetpoint(rps, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void coast() {
    if (brakeMode) {
      leader.configureAsync(
          coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      for (SparkMax follower : followers) {
        follower.configureAsync(
            coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    }
    brakeMode = false;
    leader.stopMotor();
  }

  @Override
  public void brake() {
    if (!brakeMode) {
      leader.configureAsync(
          brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      for (SparkMax follower : followers) {
        follower.configureAsync(
            brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    }
    brakeMode = true;
    leader.stopMotor();
  }

  @Override
  public void resetPosition(Angle angle) {
    leaderEncoder.setPosition(angle.in(positionUnit));
  }

  @Override
  public int getNumFollowers() {
    return followers.length;
  }

  public EncoderIO getAbsoluteEncoder(AngleUnit encoderUnit) {
    AbsoluteEncoder encoder = leader.getAbsoluteEncoder();
    return (inputs) -> {
      inputs.connected = !leader.hasActiveFault();
      inputs.absolutePosition = encoderUnit.of(encoder.getPosition());
    };
  }
}
