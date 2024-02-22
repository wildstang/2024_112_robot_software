package org.wildstang.year2024.subsystems.armPivot;

public final class ArmConstants {

    public static final double ZERO_OFFSET = 29.0;

    public static final double kP = 0.1;

    public static final double kI = 0.01;

    public static final double kD = 0;

    public static final double kF = 1/(187.5 * 23.01194 * 2); // gear ratio 187.5, neo stall torque 23.01194 in*lbs, 2 motors
    
}
