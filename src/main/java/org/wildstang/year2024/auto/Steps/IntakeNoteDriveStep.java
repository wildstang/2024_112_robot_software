package org.wildstang.year2024.auto.Steps;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.google.gson.Gson;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class IntakeNoteDriveStep extends AutoStep{
    private ShooterSubsystem shooter;
    private SwerveDrive drive;
    private ChoreoTrajectory pathtraj;
    private boolean isBlue;

    private ChoreoTrajectoryState localAutoState;

    private Timer timer;

    private ChassisSpeeds localAutoVel;

    private Pose2d drivePose;

    public IntakeNoteDriveStep(String pathData, boolean isBlue) {
        
        this.pathtraj = getTraj(pathData);
        
        this.isBlue = isBlue;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        shooter.setShooterState(shooterType.FLOOR_INTAKE);
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        
    }

    @Override
    public void update() {
        if (shooter.isIdle()){
        setFinished();
        } 
        if (timer.get() >= pathtraj.getTotalTime() && !drive.noteInView()) {
            double heading = ((2.0 * Math.PI) + pathtraj.getFinalPose().getRotation().getRadians()) % (2.0 * Math.PI);
            if (!isBlue) heading = (2.0 * Math.PI + (Math.PI - heading)) % (2.0 * Math.PI);
            drive.setAutoValues(0.0, 0.0, 0.0, 0.0, 0.0, heading);
            setFinished();
        } else if (drive.noteInView()) {
            drive.autoNoteAim(true);
        } else {
            localAutoState = pathtraj.sample(timer.get(), !isBlue);
            localAutoVel = ChassisSpeeds.discretize(localAutoState.getChassisSpeeds(), 0.02);
            drivePose = drive.returnPose();
            //update values the robot is tracking to
            drive.setAutoValues(localAutoVel.vxMetersPerSecond, localAutoVel.vyMetersPerSecond, localAutoVel.omegaRadiansPerSecond, localAutoState.x - drivePose.getX(), localAutoState.y - drivePose.getY(), localAutoState.heading);
        }
    }

    @Override
    public String toString() {
        return "Intake note";
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
