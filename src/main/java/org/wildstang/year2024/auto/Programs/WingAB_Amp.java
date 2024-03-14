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


public class WingAB_Amp extends AutoProgram{
    
    private boolean color = true;

    @Override
    protected void defineSteps(){
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().equals(Alliance.Blue));

        addStep(new StartOdometryStep(swerve.returnPose().getX(), swerve.returnPose().getY(),225, color));
        addStep(new SetGyroStep(-2.245537422197309, swerve));
        addStep(new ShootNoteStep());

        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("WingAB_Amp.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("WingAB_Amp.2", swerve, color));
        addStep(group1);
        addStep(new ShootNoteStep());




    }

    public String toString(){
        return "WingAB Program";
    }
}
