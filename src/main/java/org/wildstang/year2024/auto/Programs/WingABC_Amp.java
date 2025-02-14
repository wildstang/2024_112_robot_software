package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class WingABC_Amp extends AutoProgram{
    
    private boolean color = true;

    @Override
    protected void defineSteps(){
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().orElse(null).equals(Alliance.Blue));

        //Preload
        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));
        addStep(new ShootNoteStep());

        // Get and Shoot Wing A
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("WingAB_Amp.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());

        // Get and Shoot Wing B
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("WingAB_Amp.2", swerve, color));
        addStep(group1);
        addStep(new ShootNoteStep());

        // Get and Shoot Wing C
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("WingABC_Amp.3", swerve, color));
        addStep(group2);
        addStep(new ShootNoteStep());

    }

    public String toString(){
        return "WingABC Amp";
    }
}
