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

public class FullAutoTop extends AutoProgram{

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
       group0.addStep(new intakeNoteStep(true));
       group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Wing1", new PathConstraints(4.0, 3.0)), swerve, color));

       // Rocket
       AutoParallelStepGroup group1 = new AutoParallelStepGroup();
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 210.120823901, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));
       group1.addStep(new intakeNoteStep(true);
       group1.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle1", new PathConstraints(4.0, 3.0)), swerve, color));
       
       // Shoot
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 180, color));
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle1", new PathConstraints(4.0, 3.0)), swerve, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));

       //Middle 2
       AutoParallelStepGroup group2 = new AutoParallelStepGroup();
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 199.093485079, color));
       group2.addStep(new intakeNoteStep(true));
       group2.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle2", new PathConstraints(4.0, 3.0)), swerve, color));

       //Shoot Middle 2
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 133.71980388435017062, color));
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle2", new PathConstraints(4.0, 3.0)), swerve, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));

       // Middle 3
       AutoParallelStepGroup group3 = new AutoParallelStepGroup();
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 169.69518559871849561, color));
       group3.addStep(new intakeNoteStep(true));
       group3.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle3", new PathConstraints(4.0, 3.0)), swerve, color));

       //ShootMiddle 3
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 179.37728434205416761, color));
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle3", new PathConstraints(4.0, 3.0)), swerve, color));
       addStep(new ShootNoteStep(true));
       addStep(new AutoStepDelay(500));
       addStep(new ShootNoteStep(false));



         
    }

    @Override
    public String toString() {
        return "Full Auto Top";
    }
}
