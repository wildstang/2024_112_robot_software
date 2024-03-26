package org.wildstang.year2024.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SetGyroStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.auto.Steps.ShootNoteStep;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PreloadAuto extends AutoProgram {
    private boolean color;
    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        color = (DriverStation.getAlliance().orElse(null).equals(Alliance.Blue));
        // Shoot Preload
        addStep(new SetGyroStep(swerve.getPosTheta(), swerve, color));
        addStep(new ShootNoteStep());
     
    }

    @Override
    public String toString() {
        return "Preload Auto";
    }
    
}
