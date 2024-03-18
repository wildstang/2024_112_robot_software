package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.PathHeadingStep;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TestProgram extends AutoProgram{

    private boolean color;
    
    protected void defineSteps(){
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().orElse(null).equals(Alliance.Blue));

        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new PathHeadingStep(Math.PI, swerve));
        addStep(group1);
        addStep(new SwervePathFollowerStep("ChoreoTest", swerve, true));
    }

    public String toString(){
        return "Test Program";
    }
}
