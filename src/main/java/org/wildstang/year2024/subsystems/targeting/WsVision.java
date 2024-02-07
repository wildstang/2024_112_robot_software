package org.wildstang.year2024.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2024.robot.WsInputs;

import java.util.Optional;

import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsVision implements Subsystem {

    public LimeConsts LC;

    public Optional<Alliance> station; 

    ShuffleboardTab tab = Shuffleboard.getTab("Tab");

    public boolean TargetInView(){
        return left.TargetInView() || right.TargetInView();
    }

    //get ySpeed value for auto drive
    /*public double getScoreY(double offset){
        if (right.TargetInView() && left.TargetInView()){
            return LC.VERT_AUTOAIM_P * (offset*LC.OFFSET_VERTICAL + (getLeftVertical() + getRightVertical())/2.0);
        } else if (left.tv > 0.0){
            return (getLeftVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        } else {
            return (getRightVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        }
    }*/

    // Get x Pos and y Pos and calulate angle of turn needed to line up with speaker
    public double getAngleToSpeaker(){
        double xPosition;
        double yPosition;
        double angleToSpeaker;

        try{
            if(station.get().equals(Alliance.Blue)){
                xPosition = LC.BLUE_SPEAKER_X - left.blue3D[0];
                yPosition = -left.blue3D[1];

                angleToSpeaker = Math.atan(xPosition/yPosition);
                
                
            }else if(station.get().equals(Alliance.Red)){
                xPosition = LC.RED_SPEAKER_X - left.red3D[0];
                yPosition = -left.red3D[1];
                
                angleToSpeaker = Math.atan(xPosition/yPosition);
            }
        }catch(Exception e){
            return 0;
        }
            return angleToSpeaker;
    }

    // Get the distance the robot is from AMP within a 49 inch radius and return the angle and direction the robot needs to drive
    public boolean isInAmpRadius(){

        double ID = left.getAprilID(); // ID of AprilTags
        double robotDistance; // Distance of robot from AprilTag

        // Red Alliance April Tag
        if(ID == 5||ID == 6){
            try{
            if(station.get().equals(Alliance.Red)){
                robotDistance = Math.sqrt((Math.pow((0 - left.red3D[0]),2)) + (Math.pow((LC.AMP_Y - left.red3D[1]),2)));
            }else if(station.get().equals(Alliance.Blue)){
                robotDistance = Math.sqrt((Math.pow((LC.FIELD_WIDTH - left.red3D[0]),2)) + (Math.pow((LC.AMP_Y - left.red3D[1]),2)));
            }
            
            if(robotDistance <= LC.RADIUS_OF_AMP_TARGETING_ZONE){
                return true;
            }else{
                return false;
            }
        }catch(Exception e){
            return false;
        }

    }

    public double getDistanceToCenterOfChainPlusOffset(){
        double robotDriveDistance;
        try{
        if(station.get().equals(Alliance.Blue)){
            double xPos = left.blue3D[0];
            double yPos = left.blue3D[1];
            double aprilTagX = left.getTagX();
            double aprilTagY = left.getTagY();
            double robotDistance = Math.sqrt(Math.pow(xPos - aprilTagX,2) + Math.pow(yPos - aprilTagY,2));
            double angleAtAprilTag = 0;
            robotDriveDistance = 
                Math.sqrt((
                    (Math.pow(
                        (LC.CORE_OF_STAGE_TO_CHAIN + LC.CLIMBER_OFFSET),2)
                    ) + 
                    (Math.pow(robotDistance,2)) - (2*((LC.CORE_OF_STAGE_TO_CHAIN + LC.CLIMBER_OFFSET)))) * Math.cos(Math.toRadians(angleAtAprilTag)))
        }
    }catch(Exception e){
        return 0;
    }
        return robotDriveDistance;
        

    }

    public double[] FindThirdVertex(double sideA, double sideB, double sideC, double[] vertex1, double[] vertex2){
        double angleA = Math.toDegrees(Math.acos(((sideB*sideB)+(sideC*sideC) - (sideA*sideA)) / (2*sideB*sideC))); //Degrees
        double angleB = Math.toDegrees(Math.acos(((sideA*sideA)+(sideC*sideC) - (sideB*sideB)) / (2*sideA*sideC))); // Degrees
        double angleC = Math.toDegrees(Math.acos(((sideB*sideB)+(sideA*sideA) - (sideC*sideC)) / (2*sideA*sideB))); // Degrees
        
        double directionX = Math.cos(Math.toRadians(angleA));
        double directionY = Math.sin(Math.toRadians(angleA));

        double thirdVertexX = vertex2[0] + sideA * directionX;
        double thirdVertexY = vertex2[1] + sideA * directionY;

        double[] thirdVertex = {thirdVertexX, thirdVertexY};

        return thirdVertex;
    
    }

    /*public double[] getPointOfCenterChain(){
        
    }
*/
    
    public double getPosX(){
    try{
        if (station.get().equals(Alliance.Red)){
            return (double)(left.red3D[0]);
        }else if(station.get().equals(Alliance.Blue)){
            return (double)(left.blue3D[0]);
        }
    }catch(Exception e){
        return 0;
    }
    }

    public double getPosY(){
        try{
        if (station.get().equals(Alliance.Red)){
            return left.red3D[1];
        }else if(station.get().equals(Alliance.Blue)){
            return left.blue3D[1];
        }
    }catch(Exception e){
        return 0;
    }

    }

    public double getDistanceFromAprilTag(){
        return Math.sqrt(Math.pow(left.getTagX() - getPosX(),2) + Math.pow((left.getTagY() - getPosY()),2));
    }
    
    

    //get ySpeed value for station auto drive
    /*public double getStationY(double offset){
        if (left.TargetInView()){
            return (offset*LC.STATION_OFFSETS + (left.tid > 4.5 ? -left.blue3D[0] + LC.STATION_VERTICAL 
                : -left.red3D[0] + LC.STATION_VERTICAL)) * -LC.VERT_AUTOAIM_P;
        } else {
            return (offset*LC.STATION_OFFSETS + (right.tid > 4.5 ? -right.blue3D[0] + LC.STATION_VERTICAL 
                : -right.red3D[0] + LC.STATION_VERTICAL)) * -LC.VERT_AUTOAIM_P;
        }
    }*/
    //get xSpeed value for station auto drive
    /*public double getStationX(double offset){
        if (left.TargetInView()){
            if (left.isSeeingBlue()){
                //basically this garbage line determines whether we're going to the left or right double substation
                //  by which one's closer, adds on the driver offset, and then calculates the xSpeed for that
                //  desired displacement.
                return -LC.HORI_AUTOAIM_P * (-offset*LC.STATION_OFFSETS + (left.blue3D[1] - LC.BLUE_STATION_X - 
                    LC.STATION_HORIZONTAL*Math.signum(left.blue3D[1]-LC.BLUE_STATION_X)));
            } else {
                return -LC.HORI_AUTOAIM_P * (-offset*LC.STATION_OFFSETS + (left.red3D[1] - LC.RED_STATION_X - 
                LC.STATION_HORIZONTAL*Math.signum(left.red3D[1]-LC.RED_STATION_X)));
            }
        } else {
            if (right.isSeeingBlue()){
                return -LC.HORI_AUTOAIM_P * (-offset*LC.STATION_OFFSETS + (right.blue3D[1] - LC.BLUE_STATION_X - 
                LC.STATION_HORIZONTAL*Math.signum(right.blue3D[1]-LC.BLUE_STATION_X)));
            } else {
                return -LC.HORI_AUTOAIM_P * (-offset*LC.STATION_OFFSETS + (right.red3D[1] - LC.RED_STATION_X - 
                LC.STATION_HORIZONTAL*Math.signum(right.red3D[1]-LC.RED_STATION_X)));
            }
        }
    }*/
    //gets the vertical distance to target for grid targets from the left limelight
    /*private double getLeftVertical(){
        if (left.isSeeingBlue()){
            return -left.blue3D[0] + (LC.VERTICAL_APRILTAG_DISTANCE + (isLow ? 10.0 : 0.0)); 
        } else {
            return -left.red3D[0] + (LC.VERTICAL_APRILTAG_DISTANCE + (isLow ? 10.0 : 0.0));
        }
    }
    //gets the vertical distance to target for grid targets from the right limelight
    private double getRightVertical(){
        if (right.isSeeingBlue()){
            return -right.blue3D[0] + (LC.VERTICAL_APRILTAG_DISTANCE); 
        } else {
            return -right.red3D[0] + (LC.VERTICAL_APRILTAG_DISTANCE);
        }
    }

    //get xSpeed value for autodrive
    public double getScoreX(double offset){
        if (right.TargetInView() && left.TargetInView()){
            return LC.HORI_AUTOAIM_P * (offset*LC.OFFSET_HORIZONTAL + (getLeftHorizontal() + getRightHorizontal())/2.0);
        } else if (left.TargetInView()){
            return (getLeftHorizontal()+offset*LC.OFFSET_HORIZONTAL) * LC.HORI_AUTOAIM_P;
        } else {
            return (getRightHorizontal()+offset*LC.OFFSET_HORIZONTAL) * LC.HORI_AUTOAIM_P;
        }
    }
    //get the horizontal distance to targets on the grid from the left limelight
   /* */

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void init() {
        LC = new LimeConsts();

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        left.update();
        right.update();
        SmartDashboard.putBoolean("limelight target in view", TargetInView());
        station = DriverStation.getAlliance();
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}