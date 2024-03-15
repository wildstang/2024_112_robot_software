package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
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

public class QualsAuto extends AutoProgram{
    private boolean color = true;
    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().equals(Alliance.Blue));
        //Preload 
        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 0, color));
        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));
        addStep(new SwervePathFollowerStep("Quals.1", swerve, color));
        addStep(new ShootNoteStep());

        //Get Wing A
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("Quals.2", swerve, color));
        addStep(group0);

        //Shoot Wing A
        addStep(new SwervePathFollowerStep("Quals.3", swerve, color));
        addStep(new ShootNoteStep());

        //Get Wing B
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("Quals.4", swerve, color));
        addStep(group1);
    
        //Score wing B
        addStep(new ShootNoteStep());

        //get wing C
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("Quals.5", swerve, color));
        addStep(group2);
        //shoot wing c
        addStep(new ShootNoteStep());

    //get center D
    AutoParallelStepGroup group3 = new AutoParallelStepGroup();
    group3.addStep(new IntakeNoteStep());
    group3.addStep(new SwervePathFollowerStep("Quals.6", swerve, color));

    //go shoot center D
    addStep(new SwervePathFollowerStep("Quals.7", swerve, finished));
    addStep(new ShootNoteStep());
    }
    @Override
    public String toString() {
        return "Quals Auto";
    }
    
}
