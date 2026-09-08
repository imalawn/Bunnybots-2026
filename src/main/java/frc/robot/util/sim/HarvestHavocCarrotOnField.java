package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class HarvestHavocCarrotOnField extends GamePieceOnFieldSimulation {
  public static GamePieceOnFieldSimulation.GamePieceInfo HARVEST_HAVOC_CARROT_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          "Carrot",
          new Rectangle(Units.inchesToMeters(6), Units.inchesToMeters(18)),
          Inches.of(6),
          Kilograms.of(0.294),
          0.5,
          0.5,
          0.15);

  public HarvestHavocCarrotOnField(Pose2d initialPose) {
    super(HARVEST_HAVOC_CARROT_INFO, initialPose);
  }
}
