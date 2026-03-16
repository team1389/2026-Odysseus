// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);

  public static final Pose2d blueHub =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.845)),
          new Rotation2d(Units.degreesToRadians(180)));
}
