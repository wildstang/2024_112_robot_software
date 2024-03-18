package org.wildstang.framework.auto.steps;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import com.google.gson.Gson;
import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import com.choreo.lib.*;

public class SwervePathFollowerStep extends AutoStep {

    private SwerveDriveTemplate m_drive;
    private ChoreoTrajectory pathtraj;
    private boolean isBlue;

    private ChoreoTrajectoryState localAutoState;

    private Timer timer;

    private ChassisSpeeds localAutoVel;

    /** Sets the robot to track a new path
     * finishes after all values have been read to robot
     * @param pathData double[][] that contains path, should be from \frc\paths
     * @param drive the swerveDrive subsystem
     * @param isBlue whether the robot is on the blue alliance
     */
    public SwervePathFollowerStep(String pathData, SwerveDriveTemplate drive, boolean isBlue) {
        
        this.pathtraj = getTraj(pathData);
        m_drive = drive;
        
        this.isBlue = isBlue;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        //start path
        m_drive.setToAuto();
        timer.start();
    }

    @Override
    public void update() {
        if (timer.get() >= pathtraj.getTotalTime()) {
            m_drive.setAutoValues(0.0, 0.0, 0.0, pathtraj.getFinalPose().getX(), pathtraj.getFinalPose().getY(), pathtraj.getFinalPose().getRotation().getRadians());
            setFinished();
        } else {
            localAutoState = pathtraj.sample(timer.get(), !isBlue);
            localAutoVel = ChassisSpeeds.discretize(localAutoState.getChassisSpeeds(), 0.02);
            //update values the robot is tracking to
            m_drive.setAutoValues(localAutoVel.vxMetersPerSecond, localAutoVel.vyMetersPerSecond, localAutoVel.omegaRadiansPerSecond, localAutoState.x, localAutoState.y, localAutoState.heading);
        }
    }

    @Override
    public String toString() {
        return "Swerve Path Follower";
    }

    public ChoreoTrajectory getTraj(String fileName){
        Gson gson = new Gson();
        var tempfile = Filesystem.getDeployDirectory();
        var traj_dir = new File(tempfile, "choreo");

        var traj_file = new File(traj_dir, fileName + ".traj");
        try {
        var reader = new BufferedReader(new FileReader(traj_file));
            return  gson.fromJson(reader, ChoreoTrajectory.class);
        } catch (Exception ex) {
            DriverStation.reportError("Error loading choreo file", ex.getStackTrace());
        }
        return new ChoreoTrajectory();
    }
}

