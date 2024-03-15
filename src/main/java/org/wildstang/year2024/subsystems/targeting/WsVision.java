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

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.wildstang.framework.io.inputs.Input;

public class WsVision implements Subsystem {

    public PhotonCamera frontCam, rearCam;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator frontPoseEstimator,rearPoseEstimator;
    public EstimatedRobotPose frontPose, rearPose;

    @Override
    public void inputUpdate(Input source) {
    }

   
    @Override
    public void init() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //Forward Camera
        frontCam = new PhotonCamera("RearCam");
        Transform3d robotToFrontCam = new Transform3d(new Translation3d(0.2823972, -0.2571242, 0.6094984), new Rotation3d(0,-0.6108,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, robotToFrontCam);

        //Forward Camera
        rearCam = new PhotonCamera("FrontCam");
        Transform3d robotToRearCam = new Transform3d(new Translation3d(0.2827528, 0.256921, 0.6094984), new Rotation3d(0,0.5326,-3.1416)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCam, robotToRearCam);

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        frontPose = frontPoseEstimator.update().orElse(null);
        if (frontPose != null){
            double[] fpose = {frontPose.estimatedPose.getX(),frontPose.estimatedPose.getY(),frontPose.estimatedPose.getZ()};
            SmartDashboard.putNumberArray("front pose", fpose);
        }
        rearPose = frontPoseEstimator.update().orElse(null);
        if (rearPose != null){
            double[] rpose = {rearPose.estimatedPose.getX(),rearPose.estimatedPose.getY(),rearPose.estimatedPose.getZ()};
            SmartDashboard.putNumberArray("rear pose", rpose);
        }
    }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(frontPose != null){
            estimator.addVisionMeasurement(frontPose.estimatedPose.toPose2d(), frontPose.timestampSeconds);
        }
        if(rearPose != null){
            estimator.addVisionMeasurement(rearPose.estimatedPose.toPose2d(), rearPose.timestampSeconds);
        }
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}