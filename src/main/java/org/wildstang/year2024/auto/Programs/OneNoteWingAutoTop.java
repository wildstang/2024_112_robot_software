package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class OneNoteWingAutoTop extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().orElse(null).equals(Alliance.Blue));

        //Preload Shot
        addStep(new StartOdometryStep(swerve.getPosX(),swerve.getPosY(), 180, color));//defines position and angle the robot is currently in (used to estimate)
        addStep(new ShootNoteStep());
        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));

        // // Wing Note 1
        // AutoParallelStepGroup group0 = new AutoParallelStepGroup();
        // group0.addStep(new IntakeNoteStep());
        // group0.addStep(new SwervePathFollowerStep("OneNoteWingAutoTop.1", swerve, color));//gets values to drive toward
        // addStep(group0);
        // addStep(new ShootNoteStep());

    }

    @Override
    public String toString() {
        return "One Note Wing Auto Top";
    }
}
