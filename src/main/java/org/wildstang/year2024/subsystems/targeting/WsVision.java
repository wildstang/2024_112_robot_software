package org.wildstang.year2024.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class WsVision implements Subsystem {

    public PhotonCamera frontCam, rearCam;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator frontPoseEstimator,rearPoseEstimator;
    public EstimatedRobotPose frontPose, rearPose;
    boolean hasTargets;

    public Optional<Alliance> station;

   
    
    @Override
    public void inputUpdate(Input source) {
    }

   
    @Override
    public void init() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //Forward Camera
        frontCam = new PhotonCamera("photonvision");
        Transform3d robotToFrontCam = new Transform3d(new Translation3d(0.2823972, -0.2571242, 0.6094984), new Rotation3d(0,-0.488692,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, robotToFrontCam);

        //Forward Camera
        rearCam = new PhotonCamera("photonvision");
        Transform3d robotToRearCam = new Transform3d(new Translation3d(0.2827528, 0.256921, 0.6094984), new Rotation3d(0,0.488692,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCam, robotToRearCam);

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        var result = frontCam.getLatestResult();
        hasTargets = result.hasTargets();
        SmartDashboard.putBoolean("hasTargets", hasTargets);

        if(hasTargets) {
            // target = null;
            // targets = result.targets;
            // for(int i=0;i<targets.size();i++){
            //     target = targets.get(i);
            //     if (target.getFiducialId() == 4 || target.getFiducialId() == 7) break;
            // }
            // if (target.getFiducialId() == 4 || target.getFiducialId() == 7){
            //     yaw = target.getYaw();
            //     SmartDashboard.putBoolean("has AprilTag target", true);
            // } else {
            //     yaw = Double.NaN;
            //     SmartDashboard.putBoolean("has AprilTag target", false);
            // }
            // SmartDashboard.putNumber("yaw", yaw);
            frontPose = frontPoseEstimator.update().orElse(null);
            if(frontPose != null){
                double[] pose = {frontPose.estimatedPose.getX(),frontPose.estimatedPose.getY(),frontPose.estimatedPose.getZ()};
                SmartDashboard.putNumberArray("front pose", pose);
            } else {
                double[] pose = {-1,-1,-1};
                SmartDashboard.putNumberArray("front pose", pose);
            }
            rearPose = frontPoseEstimator.update().orElse(null);
            if(rearPose != null){
                double[] pose = {rearPose.estimatedPose.getX(),rearPose.estimatedPose.getY(),rearPose.estimatedPose.getZ()};
                SmartDashboard.putNumberArray("rear pose", pose);
            } else {
                double[] pose = {-1,-1,-1};
                SmartDashboard.putNumberArray("rear pose", pose);
            }
        } 
        // else {
        //     SmartDashboard.putNumber("pose", 0);
        // }
    }

    // public double getYaw(){
    //     return yaw;
    // }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(frontPose != null){
            estimator.addVisionMeasurement(frontPose.estimatedPose.toPose2d(), frontPose.timestampSeconds);
        }
        if(rearPose != null){
            estimator.addVisionMeasurement(rearPose.estimatedPose.toPose2d(), rearPose.timestampSeconds);
        }
    }

    // public double getYaw(){
    //     return yaw;
    // }

    @Override
    public void resetState() {
        hasTargets = false;
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}