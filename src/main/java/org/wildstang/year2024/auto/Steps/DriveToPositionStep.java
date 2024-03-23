package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToPositionStep extends AutoStep{
    private SwerveDriveTemplate driveTrain;
    private double xError, yError;
    private Pose2d goalPose, currentPose;

    public DriveToPositionStep(SwerveDriveTemplate drive, Pose2d autoGoalPose){
        driveTrain = drive;
        goalPose = autoGoalPose;
    }

    @Override
    public void initialize() {
        driveTrain.setToAuto();
    }

    @Override
    public void update() {
        currentPose = driveTrain.returnPose();
        xError = goalPose.getX() - currentPose.getX();
        yError = goalPose.getY() - currentPose.getY();
        if(xError < 0.1 && yError < 0.1){
            setFinished();
        }
        driveTrain.setAutoValues(0.0, 0.0, 0.0, goalPose.getX(), goalPose.getY(), goalPose.getRotation().getRadians());

    }

    @Override
    public String toString() {
        return "Drive to Position";
    }
    
}