package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class StartOdometryStep extends AutoStep{

    private double x, y, heading;
    private SwerveDrive swerve;

    public StartOdometryStep(double X, double Y, double pathHeading){
        x = X;
        y = Y;
        heading = pathHeading;
    }
    public void update(){
        swerve.setPose(new Pose2d(new Translation2d(x, y), new Rotation2d(heading)));
        this.setFinished();
    }
    public void initialize(){
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }
    public String toString(){
        return "Start Odometry";
    }
    
}
