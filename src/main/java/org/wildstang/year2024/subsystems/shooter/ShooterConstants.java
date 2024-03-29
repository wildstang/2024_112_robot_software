package org.wildstang.year2024.subsystems.shooter;

public class ShooterConstants {
    public static final double RATIO = 30.0 / 18.0;
    public static final double MAX_VEL = 5040 * RATIO * 2 * Math.PI / 60.0;  // rad/s
    public static final double VEL_DB = 30;  // rad/s TODO: verify deadband

    public static final double kP = 0.003;
    public static final double kF = 1.0 / MAX_VEL;
    
    public static final double AMP_SPEED = 225;  // rad/s
    public static final double SPEAKER_SPEED = 600;  //782->675 rad/s TODO: verify speed
    public static final double OUTTAKE_SPEED = -300;  // rad/s TODO: verify speed
    public static final double SOURCE_INTAKE_SPEED = -300;  // rad/s
}
