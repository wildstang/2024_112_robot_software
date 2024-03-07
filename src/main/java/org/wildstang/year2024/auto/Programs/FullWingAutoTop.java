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

public class FullWingAutoTop extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().equals(Alliance.Blue));

        //Preload Shot
        addStep(new StartOdometryStep(swerve.getPosX(),swerve.getPosY(), 0, color));
        addStep(new SwervePathFollowerStep(PathPlanner.loadPath("FullWing-PreLoad", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(new ShootNoteStep());
        
        // Wing Note 1
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 210.120823901, color));
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("FullWing-FirstNote", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());
        
        //Wing Note 2
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 210.120823901, color));
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("FullWing-SecondNote", new PathConstraints(4.0, 3.0)), swerve, color));
        addStep(group1);
        addStep(new ShootNoteStep());

       //Wing Note 3
       addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(),180, color));
       AutoParallelStepGroup group2 = new AutoParallelStepGroup();
       group2.addStep(new IntakeNoteStep());
       group2.addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 178.31533352936483539, color));
addStep(group2);
       addStep(new ShootNoteStep());;
    }

    @Override
    public String toString() {
        return "Full Wing Auto Top";
    }
}
