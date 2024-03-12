package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.auto.Steps.ScoreAmp;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class QualsPlus extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().equals(Alliance.Blue));

        addStep(new StartOdometryStep(swerve.getPosX(), swerve.getPosY(),225, color));
        addStep(new SetGyroStep(-2.245537422197309, swerve));
       //addStep(new SwervePathFollowerStep("QualsPlus", swerve, color));
        addStep(new ShootNoteStep());
        
        AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        group0.addStep(new IntakeNoteStep());
        group0.addStep(new SwervePathFollowerStep("QualsPlus.1", swerve, color));
        addStep(group0);
        addStep(new ShootNoteStep());

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new IntakeNoteStep());
        group1.addStep(new SwervePathFollowerStep("QualsPlus.2", swerve, color));
        addStep(group1);

        addStep(new SwervePathFollowerStep("QualsPlus.3", swerve, color));
        addStep(new ShootNoteStep());

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("QualsPlus.4", swerve, color));
        addStep(group2);

        addStep(new SwervePathFollowerStep("QualsPlus.5", swerve, color));
        addStep(new ShootNoteStep());







    }

    @Override
    public String toString() {
        return "Quals Plus";
    }
}
