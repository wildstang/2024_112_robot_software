package org.wildstang.year2024.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;

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
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wildstang.framework.io.inputs.Input;

public class WsVision implements Subsystem {

    public PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    public EstimatedRobotPose curPose;
    boolean hasTargets;
    PhotonTrackedTarget target;
    double yaw;

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
            target = result.getBestTarget();
            yaw = target.getYaw();
            SmartDashboard.putNumber("yaw", yaw);
            curPose = photonPoseEstimator.update().orElse(null);
            if(curPose != null){
                double[] pose = {curPose.estimatedPose.getX(),curPose.estimatedPose.getY(),curPose.estimatedPose.getZ()};
                SmartDashboard.putNumberArray("pose", pose);
            }
        } else {
            SmartDashboard.putNumber("pose", 0);
        }
    }

    public double getYaw(){
        return yaw;
    }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(curPose != null){
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