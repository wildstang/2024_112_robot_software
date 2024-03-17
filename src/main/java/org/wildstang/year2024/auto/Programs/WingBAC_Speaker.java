package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.PathHeadingStep;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class WingBAC_Speaker extends AutoProgram{
    
    private boolean color = true;

    @Override
    protected void defineSteps(){
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().equals(Alliance.Blue));

         // Preload From Subwoofer
        addStep(new StartOdometryStep(swerve.returnPose().getX(), swerve.returnPose().getY(),180, color));
        addStep(new SetGyroStep(Math.PI, swerve, color));
        addStep(new ShootNoteStep());

        // Get and Shoot Wing B
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("WingBAC_Speaker.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());

        // Get And Shoot Wing A
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("WingBAC_Speaker.2", swerve, color));
        addStep(group1);
        addStep(new ShootNoteStep());

        // Get and Shoot Wing C
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("WingBAC_Speaker.3", swerve, color));
        addStep(group2);
        addStep(new ShootNoteStep());



    }

    public String toString(){
        return "WingBAC From Speaker Program";
    }
}
