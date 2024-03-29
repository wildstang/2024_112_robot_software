package org.wildstang.year2024.subsystems.swerve;

public final class DriveConstants {

    /** robot length from swerve pod to swerve pod, in meters */
    public static final double ROBOT_LENGTH = 0.5334;
    /** robot width from swerve pod to swerve pod, in meters */
    public static final double ROBOT_WIDTH = 0.5334;
    /**speed with which the robot rotates relative to drive speed */
    public static final double ROTATION_SPEED = 0.5;
    /**offset of module 1, the front left module, in degrees */
    public static final double FRONT_LEFT_OFFSET = Math.PI / 2.0;
    /**offset of module 2, the front right module, in degrees */
    public static final double FRONT_RIGHT_OFFSET = 0;
    /**offset of module 3, the rear left module, in degrees */
    public static final double REAR_LEFT_OFFSET = Math.PI;
    /**offset of module 4, the rear right module, in degrees */
    public static final double REAR_RIGHT_OFFSET = 3.0 * Math.PI / 2.0;
    /**deadband of the controller's joysticks */
    public static final double DEADBAND = 0.07;
    /**factor of thrust for the drive trigger */
    public static final double DRIVE_THRUST = 0.4;
    /**factor of derate for the drive trigger */
    public static final double DRIVE_DERATE = 2;
    /**second order correction for rotation plus driving */
    public static final double ROT_CORRECTION_FACTOR = -0.5;//0.2; update from 111 testing
    /**PID values for driveF coefficient of momentum */
    public static final double DRIVE_F_V = 0.1847;  //0.00536; updated from 111 testing, almost certainly wrong for 112
    /**PID values for drive F coefficient of kinetic friction */
    public static final double DRIVE_F_K = 0.016;
/**Feedforward value for drive rotation */
    public static final double DRIVE_F_ROT = 0.016;
    /**PID values for drive F coefficient of inertia */
    public static final double DRIVE_F_I = 0;//0.003;
    /**PID values for path tracking position error */
    public static final double POS_P = 1.5;
    /**PID values for path tracking rotation error */
    public static final double ROT_P = 0.36;
    /**Swerve Module Names */
    public static final String[] POD_NAMES = new String[]{"FL", "FR", "BL", "BR"};
    /**Deadband for deciding if drive is at position target, meters */
    public static final double POS_DB = 0.10;
}
