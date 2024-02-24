package org.wildstang.year2024.subsystems.shooter;

public class ShooterConstants {

    public static final double RATIO = 30.0 / 18.0;
    public static final double MAX_VEL = 5676 * RATIO * 2 * Math.PI / 60.0;

    public static final double kP = 0.004;
    public static final double kF = 1.0 / MAX_VEL;
    
}
