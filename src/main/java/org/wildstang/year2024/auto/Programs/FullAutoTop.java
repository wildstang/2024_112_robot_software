package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
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
        // addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Start", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());
        
        // Wing Note 1
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        // group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Wing1", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group0);

        // Rocket
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        addStep(new ShootNoteStep());
        group1.addStep(new IntakeNoteStep());
        // group1.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle1", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group1);

        // Shoot
        // addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle1", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

        //Middle 2
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        // group2.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle2", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group2);

        //Shoot Middle 2
        // addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle2", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

        // Middle 3
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new IntakeNoteStep());
        // group3.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle3", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group3);

        //ShootMiddle 3
        // addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle3", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

    }

    @Override
    public String toString() {
        return "Full Auto Top";
    }
}
