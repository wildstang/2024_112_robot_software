package org.wildstang.year2024.subsystems.shooter;

public final class ArmConstants {
public static final double SOFT_STOP_LOW = 27.0 * Math.PI/180.0; // rad
    public static final double SOFT_STOP_HIGH = 107.0 * Math.PI/180.0; // rad

    public static final double RATIO = 187.5;
    // public static final double COM = 0.3508;  // meters
    // public static final double MASS = 97.0;  // N
    // public static final double ARM_TORQUE = MASS * COM; // Nm
    // public static final double STALL_TORQUE = 3.5 * RATIO * 2;  // Nm

    // public static final double MAX_VEL = 3.09;  //5676 / RATIO *2*Math.PI/60.0 * (1-(ARM_TORQUE/STALL_TORQUE));  // max arm speed, 3.09 rad/s
    // public static final double MAX_ACC = 53.55;  //.5*(STALL_TORQUE-ARM_TORQUE) / (MASS * Math.pow(COM, 2));  // maximum acceleration, 53.55 rad/s/s
    // // public static final double MAX_DEC = -MAX_ACC;  // maximum deceleration
    // public static final double kV = 0.2642;  // (5676 / RATIO *2*Math.PI/60.0) / 12;  // motor kV adjusted for units and gear ratio, (rad/s)/volt
    // public static final double kF = 0.08; //1.0 / (5676 / RATIO * 2 * Math.PI / 60.0);
    // public static final double VEL_P = 0.0;  // 0.005;
    // public static final double DELTA_T = 0.02;  //  update rate 50 Hz = 0.02 sec update period

    public static final double POS_DB = 0.10;  // position deadband, rad  starting value (ARM_TORQUE/STALL_TORQUE)/kP
    // public static final double VEL_DB = 0.1;  // output deadband, rad/s  starting value (ARM_TORQUE/STALL_TORQUE)/kV

    public static final double kP = 1.0;
    // public static final double kI = 0.01;
    // public static final double kD = 0.4;
    // public static final double kF = MASS * COM / (RATIO * STALL_TORQUE);
    public static final double AMP_POS = 105.0 * Math.PI/180.0;  // rad
    public static final double MIN_INTAKE_POS = SOFT_STOP_LOW;  // rad
    public static final double MAX_INTAKE_POS = 75.0 * Math.PI/180.0; // rad
    public static final double SOURCE_INTAKE_POS = SOFT_STOP_HIGH;  // rad
    public static final double SUBWOOFER_POS = 54.0 * Math.PI / 180.0;
    public static final double PODIUM_POS = 35.0 * Math.PI / 180.0;
}