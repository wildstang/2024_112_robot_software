package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.auto.Steps.IntakeNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class OneNoteWingAutoTop extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().equals(Alliance.Blue));

       //Preload Shot
       addStep(new StartOdometryStep(swerve.getPosX(),swerve.getPosY(), 180, color));//defines position and angle the robot is currently in (used to estimate)
       addStep(new SwervePathFollowerStep(PathPlanner.loadPath("FullWing-PreLoad", new PathConstraints(4.0, 3.0)), swerve, color));//gets values to drive toward
       addStep(new ShootNoteStep());


       // Wing Note 1
       AutoParallelStepGroup group0 = new AutoParallelStepGroup();
       group0.addStep(new IntakeNoteStep());
       group0.addStep(new SwervePathFollowerStep(PathPlanner.loadPath("FullWing-FirstNote", new PathConstraints(4.0, 3.0)), swerve, color));
addStep(group0);
       addStep(new ShootNoteStep());

    }

    @Override
    public String toString() {
        return "One Note Wing Auto Top";
    }
}
