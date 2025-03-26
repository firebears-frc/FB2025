// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // 11z 7x 19.5y
  public static final class VisionConstants {
    public static final String kCameraName = "Arducam_OV2311_USB_Camera";
    public static final Transform3d kCameraOffset =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.625), Units.inchesToMeters(-6.75), Units.inchesToMeters(22)),
            new Rotation3d(
                Rotation2d.fromDegrees(180).getRadians(),
                Rotation2d.fromDegrees(-30.0).getRadians(),
                Rotation2d.fromDegrees(-30).getRadians()));
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
        VecBuilder.fill(1.50, 1.50, 2 * Math.PI);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.30, 0.30, Math.PI);
  }
}
