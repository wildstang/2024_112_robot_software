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

public class ElimsOneA extends AutoProgram{

    private boolean color = true;

    @Override
    protected void defineSteps() {
    
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().equals(Alliance.Blue));

        // Init and Shoot Preload
        addStep(new StartOdometryStep(swerve.returnPose().getX(), swerve.returnPose().getY(),225, color));
        if(color){
            addStep(new SetGyroStep(-2.245537422197309, swerve));
        }else{
            addStep(new SetGyroStep(-2.245537422197309+Math.PI, swerve));
        }
        
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
        addStep(new SwervePathFollowerStep("ElimsOneA.3", swerve, color));
        addStep(new ShootNoteStep());

        // Get Center B
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new IntakeNoteStep());
        group2.addStep(new SwervePathFollowerStep("ElimsOneA.4", swerve, color));
        addStep(group2);


        // Shoot Center B
        addStep(new ShootNoteStep());

        // Get Center C and Shoot
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new IntakeNoteStep());
        group3.addStep(new SwervePathFollowerStep("ElimsOneA.5", swerve, color));
        addStep(group3);
        addStep(new ShootNoteStep());

        // Get Center E and Shoot
        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new IntakeNoteStep());
        group4.addStep(new SwervePathFollowerStep("ElimsOneA.6", swerve, color));
        addStep(group4);
        addStep(new ShootNoteStep());


    




    }

    @Override
    public String toString() {
        return "ElimsOne 1a";
    }
}
