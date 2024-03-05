package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.intakeNoteStep;
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
    //Preload shot
    addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 0, color));
    addStep(new SwervePathFollowerStep(PathPlanner.loadPath("ScorePreloadAmp", new PathConstraints(4.0, 3.0)), swerve, color));
    addStep(new ShootNoteStep(true));
    addStep(new AutoStepDelay(500));
    addStep(new ShootNoteStep(false));

    //Get Wing A
    addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(), 89.999999999942702, color));
    AutoParallelStepGroup group0 = new AutoParallelStepGroup();
    group0.addStep(new intakeNoteStep(true));
    group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("GetWingA", new PathConstaints(4.0,3.0)), swerve, color));
    addStep(group0);
    addStep(new shootNoteStep(true));
    addStep(new AutoStepDelay(500));
    addStep(new shootNoteStep(false));


    



    }

    @Override
    public String toString() {
        return "Quals Auto";
    }
    
}
