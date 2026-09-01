package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.io.motors.MotorIO;

public final class IndexerConstants {
  public static final double INDEXER_KP = 0.1;
  public static final double INDEXER_KD = 0;

  public static final MotorIO.RotationalMechanismConstraints CONSTRAINTS =
      new MotorIO.RotationalMechanismConstraints(1, 0.002, 0, 0, 0, 0);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(INDEXER_KP)
                  .withKI(0)
                  .withKD(INDEXER_KD)
                  .withKS(0)
                  .withKV(0)
                  .withKA(0));

  public static final double RPS = 75;
  public static final double REVERSED_RPS = -50;

  public static final double BEAMBREAK_THRESHOLD = 75; // mm
}
