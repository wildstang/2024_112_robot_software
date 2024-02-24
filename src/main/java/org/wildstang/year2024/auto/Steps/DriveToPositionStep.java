package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.year2024.subsystems.swerve.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToPositionStep extends AutoStep{
    private SwerveDriveTemplate driveTrain;
    private double xError, yError;
    private Pose2d goalPose, currentPose;
    private double goalHeading;
    private double xVelocity,yVelocity;


    public DriveToPositionStep(SwerveDriveTemplate drive, Pose2d autoGoalPose, double goalHeading){
        driveTrain = drive;
        goalPose = autoGoalPose;
        this.goalHeading = goalHeading;
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
            driveTrain.setAutoValues(0.0, goalHeading, 0.0, 0.0, 0.0);
            setFinished();
        }
        xVelocity = xError *DriveConstants.POS_P;
        yVelocity = yError * DriveConstants.POS_P;
        driveTrain.setAutoValues(0.0, goalHeading, 0.0, xVelocity, yVelocity);

    }

    @Override
    public String toString() {
        return "Drive to Position";
    }
    
}