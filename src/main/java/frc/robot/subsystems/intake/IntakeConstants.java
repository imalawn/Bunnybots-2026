package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.Intake.PivotState.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import java.util.EnumMap;

public final class IntakeConstants {
  // setpoints, in degrees
  public static final EnumMap<Intake.PivotState, Angle> SETPOINTS =
      new EnumMap<>(Intake.PivotState.class);

  static {
    SETPOINTS.put(STOWED, Degrees.of(0.0));
    SETPOINTS.put(HANDOFF, Degrees.of(30.0));
    SETPOINTS.put(GROUND, Degrees.of(129.0));
  }

  public static final double STOW_DELAY = 1.0;

  public static final double ROLLER_RPS = 5000 / 60.0;
  public static final double ROLLER_RPS_REVERSED = -4500 / 60.0;

  // sim
  public static final int INTAKE_CAPACITY = 50;

  public static final double PIVOT_GEAR_RATIO = 75;
  public static final double PIVOT_RADIUS = 0.5; // m
  public static final double PIVOT_KP = 164.627;
  public static final double PIVOT_KD = 0.645;
  public static final double PIVOT_CRUISE_VELOCITY = 6;
  public static final double PIVOT_CRUISE_ACCELERATION = 10;

  public static final double ROLLER_MOI = 0.002;
  public static final double ROLLER_GEAR_RATIO = 1.0;
  public static final double ROLLER_KP = 0.2;
  public static final double ROLLER_KD = 0;

  public static final TalonFXConfiguration PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withFeedbackRemoteSensorID(Constants.CANConstants.INTAKE_ENCODER)
                  .withRotorToSensorRatio(20)
                  .withSensorToMechanismRatio(48.0 / 18.0))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PIVOT_KP)
                  .withKI(0)
                  .withKD(PIVOT_KD)
                  .withKS(0)
                  .withKV(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(PIVOT_CRUISE_VELOCITY)
                  .withMotionMagicAcceleration(PIVOT_CRUISE_ACCELERATION));

  public static final TalonFXConfiguration ROLLER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ROLLER_KP)
                  .withKI(0)
                  .withKD(ROLLER_KD)
                  .withKS(0)
                  .withKV(0.12));

  public static final CANcoderConfiguration ENCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(Degrees.of(0)));
}
