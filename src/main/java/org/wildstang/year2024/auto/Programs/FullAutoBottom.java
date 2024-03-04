package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FullAutoBottom extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().equals(Alliance.Blue));

        //Preload Shot
        addStep(new StartOdometryStep(swerve.getPosX(),swerve.getPosY(), 0, color));
        addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Start", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());
        
        // Rocket
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 169.83959554371344325, color));
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle1", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group1);

        // Shoot
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 169.83959554371344325, color));
        addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle1", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

        //Middle 2
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 147.07207857165727205, color));
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle2", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group2);

        //Shoot Middle 2
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 182.540434116, color));
        addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle2", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

        // Middle 3
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 166.1256132925804252, color));
        group3.addStep(new IntakeNoteStep());
        group3.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - Middle3", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group3);

        //ShootMiddle 3
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 174.40468964350796455, color));
        addStep(new SwervePathFollowerStep(PathPlanner.loadPath("Rocket - ShootMiddle3", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());

    }

    @Override
    public String toString() {
        return "Full Auto Bottom";
    }
}
