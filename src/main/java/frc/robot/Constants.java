// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum ControlScheme {
    MAIN(false),
    TEST(false),
    GUITAR_HERO_OP(true),
    GUITAR_HERO_FULL(true);

    public final boolean isGuitarHero;

    ControlScheme(boolean isGuitarHero) {
      this.isGuitarHero = isGuitarHero;
    }
  }

  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int GUITAR_HERO_CONTROLLER_PORT = 2;
    public static final double DRIVER_DEADBAND = 0.1;
    public static final double OPERATOR_DEADBAND = 0.1;
  }

  public static final class CANConstants {
    public static final CANBus SUPERSTRUCTURE = new CANBus("Superstructure");

    public static final int ELEVATOR_LEADER = 1;
    public static final int ELEVATOR_FOLLOWER = 2;
    public static final int INDEXER = 3;
    public static final int OUTTAKE = 4;
  }

  public static final class FieldConstants {
    public static final Distance FIELD_LENGTH = Meters.of(16.4592);
    public static final Distance FIELD_WIDTH = Meters.of(8.2296);
  }
}
