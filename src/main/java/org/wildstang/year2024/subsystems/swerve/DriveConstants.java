package org.wildstang.year2024.subsystems.swerve;

public final class DriveConstants {

    /** robot length from swerve pod to swerve pod, in meters */
    public static final double ROBOT_LENGTH = 0.5334;
    /** robot width from swerve pod to swerve pod, in meters */
    public static final double ROBOT_WIDTH = 0.5334;
    /**speed with which the robot rotates relative to drive speed */
    public static final double ROTATION_SPEED = 0.5;
    /**offset of module 1, the front left module, in degrees */
    public static final double FRONT_LEFT_OFFSET = -90;
    /**offset of module 2, the front right module, in degrees */
    public static final double FRONT_RIGHT_OFFSET = 0;
    /**offset of module 3, the rear left module, in degrees */
    public static final double REAR_LEFT_OFFSET = 180;
    /**offset of module 4, the rear right module, in degrees */
    public static final double REAR_RIGHT_OFFSET = 90;
    /**deadband of the controller's joysticks */
    public static final double DEADBAND = 0.05;
    /**factor of thrust for the drive trigger */
    public static final double DRIVE_THRUST = 0.4;
    /**factor of derate for the drive trigger */
    public static final double DRIVE_DERATE = 2;
    /**second order correction for rotation plus driving */
    public static final double ROT_CORRECTION_FACTOR = 0.2;
    /**PID values for driveF coefficient of momentum */
    public static final double DRIVE_F_V = 0.00536;//0.00558*1.2325;//0.00581 on old treads
    /**PID values for drive F coefficient of kinetic friction */
    public static final double DRIVE_F_K = 0.016;
    /**PID values for drive F coefficient of inertia */
    public static final double DRIVE_F_I = 0;//0.003;
    /**PID values for path tracking position error */
    public static final double POS_P = 1;
    /**PID values for path tracking rotation error */
    public static final double ROT_P = 2.5;
    /**Swerve Module Names */
    public static final String[] POD_NAMES = new String[]{"FL", "FR", "BL", "BR"};

    public static double AMP_Y = 61.5; //Inches
    public static double FIELD_WIDTH = 323.25; //INCHES

    /*Blank for now. Offset of climber which will assist in alligning the robot climber to the chain
     * With correct offset.
     */
    public double CLIMBER_OFFSET = 0; 

    public double CORE_OF_STAGE_TO_CHAIN = 16.625; //Inches
    


}
