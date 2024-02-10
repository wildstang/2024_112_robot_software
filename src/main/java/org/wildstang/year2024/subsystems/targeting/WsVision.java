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