package frc.robot.util.io.sensors;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface EncoderIO {
  @AutoLog
  class EncoderIOInputs {
    public boolean connected = false;
    public Angle absolutePosition = Rotations.zero();
  }

  void updateInputs(EncoderIOInputs inputs);
}
