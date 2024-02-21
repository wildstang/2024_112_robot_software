package org.wildstang.year2024.subsystems.targeting; 
import java.util.Dictionary;
import java.util.Hashtable;

import org.wildstang.year2024.subsystems.swerve.DriveConstants;

public class LimeConsts {

    public double FIELD_WIDTH = 323.25; //INCHES
    public double BLUE_SPEAKER_X = 36.125; //Inches
    public double RED_SPEAKER_X = FIELD_WIDTH-BLUE_SPEAKER_X; //Inches
    public double SPEAKER_Y = 218.42;
    public double ALLIANCE_LENGTH = 118.25; // INCHES
    public double CENTER_FIELD_LENGTH = FIELD_WIDTH-(ALLIANCE_LENGTH*2); // INCHES
    public int RADIUS_OF_AMP_TARGETING_ZONE = 49; //Inches
    public double AMP_X = 61.5; //Inches
    
    public WsLL aprilTags;
    
    /*Blank for now. Offset of climber which will assist in alligning the robot climber to the chain
     * With correct offset.
     */
    public double CLIMBER_OFFSET = 0; 

    public double CORE_OF_STAGE_TO_CHAIN = 16.625; //Inches

    public double[] Chain14Midpoint = {DriveConstants.tag14[0], DriveConstants.tag14[1] + CORE_OF_STAGE_TO_CHAIN};
    public double[] Chain15Midpoint = {DriveConstants.tag15[0]+((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), DriveConstants.tag15[1] + (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain16Midpoint = {DriveConstants.tag16[0]-((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), DriveConstants.tag16[1] - (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain13Midpoint = {DriveConstants.tag13[0], DriveConstants.tag14[1] + CORE_OF_STAGE_TO_CHAIN};
    public double[] Chain11Midpoint = {DriveConstants.tag11[0]+((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), DriveConstants.tag11[1] + (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain12Midpoint = {DriveConstants.tag12[0]-((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), DriveConstants.tag12[1] - (CORE_OF_STAGE_TO_CHAIN/2)};
    
}