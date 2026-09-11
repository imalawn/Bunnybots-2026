package frc.robot.util.io.sensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

public class LaserCanIOReal implements LaserCanIO {
  private final LaserCan sensor;

  public LaserCanIOReal(
      int id,
      LaserCanInterface.RangingMode rangingMode,
      LaserCanInterface.TimingBudget timingBudget,
      LaserCanInterface.RegionOfInterest roi) {
    sensor = new LaserCan(id);
    try {
      sensor.setRangingMode(rangingMode);
      sensor.setTimingBudget(timingBudget);
      sensor.setRegionOfInterest(roi);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCan Configuration failed! " + e);
    }
  }

  public LaserCanIOReal(int id) {
    this(
        id,
        LaserCanInterface.RangingMode.SHORT,
        LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS,
        new LaserCanInterface.RegionOfInterest(8, 8, 16, 16));
  }

  @Override
  public void updateInputs(LaserCanInputs inputs) {
    LaserCanInterface.Measurement measurement = sensor.getMeasurement();
    if (measurement != null) {
      inputs.connected = true;
      inputs.measurementValid =
          measurement.status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
      inputs.distanceMillimeters = measurement.distance_mm;
    } else {
      inputs.connected = false;
      inputs.measurementValid = false;
    }
  }
}
