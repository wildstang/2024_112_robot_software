package org.wildstang.year2024.subsystems.targeting; 
import java.util.Dictionary;
import java.util.Hashtable;

public class LimeConsts {

    public double RED_SPEAKER_X = 104.83; //Inches
    public double BLUE_SPEAKER_X = 218.42; //Inches
    public int RADIUS_OF_AMP_TARGETING_ZONE = 49; //Inches
    public double AMP_Y = 61.5; //Inches
    public double FIELD_WIDTH = 323.25; //INCHES
    public WsLL aprilTags;
    
    /*Blank for now. Offset of climber which will assist in alligning the robot climber to the chain
     * With correct offset.
     */
    public double CLIMBER_OFFSET = 0; 

    public double CORE_OF_STAGE_TO_CHAIN = 16.625; //Inches

    public double[] Chain14Midpoint = {aprilTags.getTagX(), aprilTags.getTagY() + CORE_OF_STAGE_TO_CHAIN};
    public double[] Chain15Midpoint = {aprilTags.getTagX()+((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), aprilTags.getTagY() + (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain16Midpoint = {aprilTags.getTagX()-((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), aprilTags.getTagY() - (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain13Midpoint = {aprilTags.getTagX(), aprilTags.getTagY() + CORE_OF_STAGE_TO_CHAIN};
    public double[] Chain11Midpoint = {aprilTags.getTagX()+((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), aprilTags.getTagY() + (CORE_OF_STAGE_TO_CHAIN/2)};
    public double[] Chain12Midpoint = {aprilTags.getTagX()-((CORE_OF_STAGE_TO_CHAIN/2)*Math.sqrt(3)), aprilTags.getTagY() - (CORE_OF_STAGE_TO_CHAIN/2)};
    
}