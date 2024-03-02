package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.intakeNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RocketAutoTop extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().equals(Alliance.Blue));

       //Preload Shot
       addStep(new StartOdometryStep(swerve.getPosX(),swerve.getPosY(), 0, color));
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Start", new PathConstraints(4.0, 3.0)), swerve, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));

       // Wing Note 1
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 215.311216827, color));
       AutoParallelStepGroup group0 = new AutoParallelStepGroup();
       group0.addStep(new intakeNoteStep(true, false));
       group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Wing1", new PathConstraints(4.0, 3.0)), swerve, color));

       // Rocket
       AutoParallelStepGroup group1 = new AutoParallelStepGroup();
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 210.120823901, color));
       group1.addStep(new ShootNoteStep(true));
       group1.addStep(new intakeNoteStep(true, false));
       group1.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle1", new PathConstraints(4.0, 3.0)), swerve, color));
       
       // Shoot
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 180, color));
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle1", new PathConstraints(4.0, 3.0)), swerve, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));







         
    }

    @Override
    public String toString() {
        return "Rocket Auto Top";
    }
}
