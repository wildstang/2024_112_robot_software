package org.wildstang.year2024.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d BLUE_SPEAKER = new Pose2d(-0.1, 5.548, new Rotation2d());  // m
    public static final Pose2d RED_SPEAKER = new Pose2d(16.64, 5.548, new Rotation2d());  // m
    public static final double SPEAKER_Z = 2.10566;  // m
    public static final Pose2d BLUE_AMP = new Pose2d(1.842, 7.811, Rotation2d.fromRadians(3.0 * Math.PI / 2.0));  // m
    public static final Pose2d RED_AMP = new Pose2d(14.630, 8.045, Rotation2d.fromRadians(3.0 * Math.PI / 2.0));  // m

    public static final double CORE_OF_STAGE_TO_CHAIN = 0.422;  // m
    public static final Pose2d Chain11 = new Pose2d(11.905 + (CORE_OF_STAGE_TO_CHAIN * 1.0 / 2.0), 3.713 - (CORE_OF_STAGE_TO_CHAIN * Math.sqrt(3.0) / 2.0), Rotation2d.fromDegrees(-Math.PI / 3.0));
    public static final Pose2d Chain12 = new Pose2d(11.905 + (CORE_OF_STAGE_TO_CHAIN * 1.0 / 2.0), 4.498 + (CORE_OF_STAGE_TO_CHAIN * Math.sqrt(3.0) / 2.0), Rotation2d.fromDegrees(Math.PI / 3.0));
    public static final Pose2d Chain13 = new Pose2d(11.220 - CORE_OF_STAGE_TO_CHAIN, 4.105, Rotation2d.fromRadians(Math.PI));
    public static final Pose2d Chain14 = new Pose2d(5.321 + CORE_OF_STAGE_TO_CHAIN, 4.105, Rotation2d.fromRadians(0.0));
    public static final Pose2d Chain15 = new Pose2d(4.641 - (CORE_OF_STAGE_TO_CHAIN * 1.0 / 2.0), 4.498 + (CORE_OF_STAGE_TO_CHAIN * Math.sqrt(3.0) / 2.0), Rotation2d.fromDegrees(2.0 * Math.PI / 3.0));
    public static final Pose2d Chain16 = new Pose2d(4.641 - (CORE_OF_STAGE_TO_CHAIN * 1.0 / 2.0), 3.713 - (CORE_OF_STAGE_TO_CHAIN * Math.sqrt(3.0) / 2.0), Rotation2d.fromDegrees(-2.0 * Math.PI / 3.0));
}
