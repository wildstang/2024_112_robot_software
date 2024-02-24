package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.StartOdometryStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TwoWingAuto extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
         SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
         color = (DriverStation.getAlliance().equals(Alliance.Blue));

       //Preload Shot
       addStep(new StartOdometryStep(1.7615007162094116,6.884628772735596, -140.1944271002657274, color));
       // addStep(new )  ##Shooting Step

       // Wing Notes
       AutoParallelStepGroup group0 = new AutoParallelStepGroup();
       group0.addStep(new StartOdometryStep(2.898200511932373, 7.020035743713379, 180, color));
       //group0.addStep(); ##Notepath Intake
       addStep(new StartOdometryStep(2.898200511932373, 7.020035743713379, -155.01896534315011422, color));
       //addStep() ##Shooter
       AutoParallelStepGroup group1 = new AutoParallelStepGroup();
       group1.addStep(new StartOdometryStep(2.8950469493865967, 5.580051422119141, 90, color));
       //group1.addStep(); ## Notepath Intake
       addStep(new StartOdometryStep(2.8950469493865967, 5.580051422119141, 180, color));
       //addStep(); ##Shooter
       


         
    }

    @Override
    public String toString() {
        return "TwoWingAuto";
    }
}
