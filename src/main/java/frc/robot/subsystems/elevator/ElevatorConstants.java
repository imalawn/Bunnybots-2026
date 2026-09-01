package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.io.motors.MotorIO;
import java.util.EnumMap;

public final class ElevatorConstants {
  public static Angle metersToRotations(Distance meters) {
    return Rotations.of(
        Units.radiansToRotations(meters.in(Meters) / CONSTRAINTS.drumRadiusMeters()));
  }

  public static final EnumMap<Elevator.Setpoint, Angle> SETPOINTS =
      new EnumMap<>(Elevator.Setpoint.class);

  static {
    SETPOINTS.put(Elevator.Setpoint.STOWED, Rotations.of(0));
    SETPOINTS.put(Elevator.Setpoint.OVEN, metersToRotations(Meters.of(0)));
    SETPOINTS.put(Elevator.Setpoint.L1, metersToRotations(Meters.of(0)));
    SETPOINTS.put(Elevator.Setpoint.L2, metersToRotations(Meters.of(0)));
  }

  public static final double SETPOINT_TOLERANCE = 0.06;

  public static final double MAX_MANUAL_VOLTAGE = 6.0;
  public static final double HOMING_VOLTAGE = 2.0;
    public static final double HOMING_VELOCITY_THRESHOLD = 0.1; // placeholder, find this

  public static final MotorIO.LinearMechanismConstraints CONSTRAINTS =
      new MotorIO.LinearMechanismConstraints(1.68, 2.0, Units.inchesToMeters(2), 0, 2);

  public static final double ELEVATOR_KP = 100;
  public static final double ELEVATOR_KD = 0;

  public static final TalonFXConfiguration MOTOR_CONFIG =
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
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(CONSTRAINTS.reduction()))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ELEVATOR_KP)
                  .withKI(0)
                  .withKD(ELEVATOR_KD)
                  .withKS(0)
                  .withKV(0)
                  .withKA(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(3)
                  .withMotionMagicAcceleration(80));
}
