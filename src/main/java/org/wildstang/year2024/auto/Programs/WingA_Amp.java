package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.IntakeNoteDriveStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class WingA_Amp extends AutoProgram{
    
    private boolean color = true;

    @Override
    protected void defineSteps(){
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue));

         // Preload
        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));
        addStep(new ShootNoteStep());

        // Get and Shoot wing A
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        // group0.addStep(new IntakeNoteDriveStep("WingA_Amp", color));
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("WingA_Amp.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());




    }

    public String toString(){
        return "WingA Amp";
    }
}
