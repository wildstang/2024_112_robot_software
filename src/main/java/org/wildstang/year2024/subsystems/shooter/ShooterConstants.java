package org.wildstang.year2024.subsystems.shooter;

public class ShooterConstants {
    public static final double RATIO = 30.0 / 18.0;
    public static final double MAX_VEL = 5676 * RATIO * 2 * Math.PI / 60.0;
    public static final double ZERO_OFFSET = 29.0;
    public static final double SOFT_STOP_HIGH = 100.0;

    public static final double kP = 0.004;
    public static final double kF = 1.0 / MAX_VEL;
    public static final double AMP_SPEED = 400 ;
}
