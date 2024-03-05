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

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class WsVision implements Subsystem {

    public PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    public EstimatedRobotPose curPose;
    boolean hasTargets;
    
    PhotonTrackedTarget target;
    double yaw;
    private List<PhotonTrackedTarget> targets;

    public Optional<Alliance> station;

   
    
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

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        var result = camera.getLatestResult();
        hasTargets = result.hasTargets();
        SmartDashboard.putBoolean("hasTargets", hasTargets);

        if(hasTargets) {
            target = null;
            targets = result.targets;
            for(int i=0;i<targets.size();i++){
                target = targets.get(i);
                if (target.getFiducialId() == 4 || target.getFiducialId() == 7) break;
            }
            if (target.getFiducialId() == 4 || target.getFiducialId() == 7){
                yaw = target.getYaw();
                SmartDashboard.putBoolean("has AprilTag target", true);
            } else {
                yaw = Double.NaN;
                SmartDashboard.putBoolean("has AprilTag target", false);
            }
            SmartDashboard.putNumber("yaw", yaw);
            curPose = photonPoseEstimator.update().orElse(null);
            if(curPose != null){
                double[] pose = {curPose.estimatedPose.getX(),curPose.estimatedPose.getY(),curPose.estimatedPose.getZ()};
                SmartDashboard.putNumberArray("pose", pose);
            }
        } 
        else {
            SmartDashboard.putNumber("pose", 0);
        }
    }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(hasTargets){
            estimator.addVisionMeasurement(curPose.estimatedPose.toPose2d(), curPose.timestampSeconds);
        }
    }

    public double getYaw(){
        return yaw;
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