package org.wildstang.year2024.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2024.robot.WsInputs;

import java.util.Optional;

import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.DigitalInput;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.NoSuchElementException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class WsVision implements Subsystem {

    public PhotonCamera camera;
    private LimeConsts lc;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    public EstimatedRobotPose curPose;
    boolean hasTargets;

    public Optional<Alliance> station;

    // Get x Pos and y Pos and calulate angle of turn needed to line up with speaker
    public double getAngleToSpeaker(){
        double xPosition;
        double yPosition;
        double angleToSpeaker;

        if(station.get().equals(Alliance.Blue)){
            xPosition = lc.BLUE_SPEAKER_X - left.blue3D[0];
            yPosition = -left.blue3D[1];

            angleToSpeaker = Math.atan(xPosition/yPosition);
            
            
        }else if(station.get().equals(Alliance.Red)){
            xPosition = lc.RED_SPEAKER_X - left.red3D[0];
            yPosition = -left.red3D[1];
            
            angleToSpeaker = Math.atan(xPosition/yPosition);
        }
        return angleToSpeaker;
    }

    // Get the distance the robot is from AMP within a 49 inch radius and return the angle and direction the robot needs to drive
    public boolean isInAmpRadius(){

        double ID = left.getAprilID(); // ID of AprilTags
        double robotDistance; // Distance of robot from AprilTag

        // Red Alliance April Tag
        if(ID == 5||ID == 6){
            if(station.orElse(null).equals(Alliance.Red)){
                robotDistance = Math.sqrt((Math.pow((0 - left.red3D[0]),2)) + (Math.pow((lc.AMP_Y - left.red3D[1]),2)));
            }else if(station.orElse(null).equals(Alliance.Blue)){
                robotDistance = Math.sqrt((Math.pow((lc.FIELD_WIDTH - left.red3D[0]),2)) + (Math.pow((lc.AMP_Y - left.red3D[1]),2)));
            }
            
            if(robotDistance <= lc.RADIUS_OF_AMP_TARGETING_ZONE){
                return true;
            }else{
                return false;
            }
        }

    }

    public double getDistanceToCenterOfChainPlusOffset(){
        double robotDriveDistance;
        
        if(station.orElse(null).equals(Alliance.Blue)){
            double xPos = left.blue3D[0];
            double yPos = left.blue3D[1];
            double aprilTagX = left.getTagX();
            double aprilTagY = left.getTagY();
            double robotDistance = Math.sqrt(Math.pow(xPos - aprilTagX,2) + Math.pow(yPos - aprilTagY,2));
            double angleAtAprilTag = 0;
            robotDriveDistance = 
                Math.sqrt((
                    (Math.pow(
                        (lc.CORE_OF_STAGE_TO_CHAIN + lc.CLIMBER_OFFSET),2)
                    ) + 
                    (Math.pow(robotDistance,2)) - (2*((lc.CORE_OF_STAGE_TO_CHAIN + lc.CLIMBER_OFFSET)))) * Math.cos(Math.toRadians(angleAtAprilTag)))
        }
        return robotDriveDistance;
        

    }

    public double[] FindThirdVertex(double sideA, double sideB, double sideC, double[] vertex1, double[] vertex2){
        double angleA = Math.toDegrees(Math.acos(((sideB*sideB)+(sideC*sideC) - (sideA*sideA)) / (2*sideB*sideC))); // Degrees
        
        double directionX = Math.cos(Math.toRadians(angleA));
        double directionY = Math.sin(Math.toRadians(angleA));

        double thirdVertexX = vertex2[0] + sideA * directionX;
        double thirdVertexY = vertex2[1] + sideA * directionY;

        double[] thirdVertex = {thirdVertexX, thirdVertexY};

        return thirdVertex;
    
    }

    public double getPosX(){
        if (station.orElse(null).equals(Alliance.Red)){
            return (double)(left.red3D[0]);
        }else if(station.orElse(null).equals(Alliance.Blue)){
            return (double)(left.blue3D[0]);
        }
    }

    public double getPosY(){
        if (station.orElse(null).equals(Alliance.Red)){
            return left.red3D[1];
        }else if(station.orElse(null).equals(Alliance.Blue)){
            return left.blue3D[1];
        }

    }

    public double getDistanceFromAprilTag(){
        return Math.sqrt(Math.pow(left.getTagX() - getPosX(),2) + Math.pow((left.getTagY() - getPosY()),2));
    }
    
    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void init() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //Forward Camera
        camera = new PhotonCamera("photonvision");
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

        lc = new LimeConsts();
        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        station = DriverStation.getAlliance();
        var result = camera.getLatestResult();
        SmartDashboard.putBoolean("hasTargets", result.hasTargets());
        hasTargets = result.hasTargets();

        if(hasTargets) {
            try{
                curPose = photonPoseEstimator.update().get();
                SmartDashboard.putNumber("pose", curPose.estimatedPose.getX());
            } catch(NoSuchElementException e) {
                SmartDashboard.putNumber("pose", 0);
            }
        } else {
            SmartDashboard.putNumber("pose", 0);
        }
    }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(hasTargets){
            estimator.addVisionMeasurement(curPose.estimatedPose.toPose2d(), curPose.timestampSeconds);
        }
    }

    @Override
    public void resetState() {
        hasTargets = false;
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}