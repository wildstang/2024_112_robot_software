package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.SetGyroCorrectStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ElimsOneA extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().orElse(null).equals(Alliance.Blue));

        addStep(new SetGyroCorrectStep(swerve));
        addStep(new ShootNoteStep());
        
        // Get and Shoot wing A
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("ElimsOneA.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());

        // Get Center A
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("ElimsOneA.2", swerve, color));
        addStep(group1);

        // Shoot CenterA
        addStep(new ShootNoteStep());

        // Get Center B
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("ElimsOneA.3", swerve, color));
        addStep(group2);
        addStep(new ShootNoteStep());

        // Get Center C and Shoot
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new IntakeNoteStep());
        group3.addStep(new SwervePathFollowerStep("ElimsOneA.4", swerve, color));
        addStep(group3);
        addStep(new ShootNoteStep());

        // Get Center D and Shoot
        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new IntakeNoteStep());
        group4.addStep(new SwervePathFollowerStep("ElimsOneA.5", swerve, color));
        addStep(group4);
        addStep(new ShootNoteStep());

    }

    @Override
    public String toString() {
        return "ElimsOneA";
    }
}
