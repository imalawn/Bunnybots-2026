// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import frc.robot.util.io.vision.EagleEyeCamera;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOEagleEye implements VisionIO {
  private final EagleEyeCamera camera;

  /**
   * Creates a new VisionIOEagleEye.
   *
   * @param key The target key of the camera in NetworkTables.
   */
  public VisionIOEagleEye(String key) {
    camera = new EagleEyeCamera("heartbeat/" + key + "/time", "localization/" + key + "/pose",
        "localization/" + key + "/meta");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<PoseObservation> poseObservations = camera.poll();
    inputs.connected = camera.isConnected();
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.isTagVisible = !poseObservations.isEmpty();
  }
}
