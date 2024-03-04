package org.wildstang.year2024.subsystems.swerve;

public class FieldConstants {
    public static final double FIELD_WIDTH = 323.25; //INCHES
    public static final double FIELD_LENGTH = 651.25; //INCHES
    public static final double BLUE_SPEAKER_X = -1.5; //Inches
    public static final double RED_SPEAKER_X = 652.73; //Inches
    public static final double SPEAKER_Y = 218.42; //inches
    // public static final double SPEAKER_Z = 218.42;//inches
    // public static final double AMP_Z = 323.00; //inches
    public static final double RED_AMP_X = 578.77; //inches
    public static final double BLUE_AMP_X = 72.5; //inches
    public static final double AMP_Y = 323.00; //inches
    // public static final double ALLIANCE_LENGTH = 118.25; //inches


    public static final double CORE_OF_STAGE_TO_CHAIN = 16.625; //Inches
    
    /*Blue Alliance Stage AprilTag Positions  [x,y]*/
    public static final double[] tag16 = {182.73, 146.19};
    public static final double[] tag15 = {182.73, 177.10};
    public static final double[] tag14 = {209.48, 161.62};

    /*Red Alliance Stage AprilTag Positions [x,y] */
    public static final double[] tag13 = {441.74, 161.62};
    public static final double[] tag12 = {468.69, 177.10};
    public static final double[] tag11 = {468.69, 146.19};

    public static final double[] Chain14Midpoint = {tag14[0], tag14[1] + CORE_OF_STAGE_TO_CHAIN};
    public static final double[] Chain15Midpoint = {tag15[0]+(CORE_OF_STAGE_TO_CHAIN*Math.sqrt(3)/2), tag15[1] + (CORE_OF_STAGE_TO_CHAIN/2)};
    public static final double[] Chain16Midpoint = {tag16[0]-(CORE_OF_STAGE_TO_CHAIN*Math.sqrt(3)/2), tag16[1] - (CORE_OF_STAGE_TO_CHAIN/2)};
    public static final double[] Chain13Midpoint = {tag13[0], tag14[1] + CORE_OF_STAGE_TO_CHAIN};
    public static final double[] Chain11Midpoint = {tag11[0]+(CORE_OF_STAGE_TO_CHAIN*Math.sqrt(3)/2), tag11[1] + (CORE_OF_STAGE_TO_CHAIN/2)};
    public static final double[] Chain12Midpoint = {tag12[0]-(CORE_OF_STAGE_TO_CHAIN*Math.sqrt(3)/2), tag12[1] - (CORE_OF_STAGE_TO_CHAIN/2)};
}
