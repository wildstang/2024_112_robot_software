package org.wildstang.year2024.robot;

/**
 * CAN Constants are stored here.
 * We primarily use CAN to communicate with Talon motor controllers.
 * These constants must correlate with the IDs set in Phoenix Tuner.
 * Official documentation can be found here:
 * https://phoenix-documentation.readthedocs.io/en/latest/ch08_BringUpCAN.html
 */
public final class CANConstants {

    //Gyro and CAN sensor values
    public static final int GYRO = 31;

    // swerve constants
    public static final int DRIVE1 = 11;
    public static final int ANGLE1 = 12;
    public static final int DRIVE2 = 13;
    public static final int ANGLE2 = 14;
    public static final int DRIVE3 = 15;
    public static final int ANGLE3 = 16;
    public static final int DRIVE4 = 17;
    public static final int ANGLE4 = 18;

    // notepath motors
    public static final int INTAKE = 19;
    public static final int FEED = 22;

    // shooter constants
    public static final int SHOOTER_ANGLE1 = 21;
    public static final int SHOOTER_ANGLE2 = 20;
    public static final int SHOOTER1 = 23;
    public static final int SHOOTER2 = 24;
    public static final int AMPHOOD = 25;
    
}