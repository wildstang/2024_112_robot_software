package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class IntakeNoteStep extends AutoStep{
    private ShooterSubsystem shooter;
    private SwerveDrive swerve;

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        swerve  = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        shooter.setShooterState(shooterType.FLOOR_INTAKE);
        swerve.autoNoteAim(true);
        Log.warn("Intake Step");
        
    }

    @Override
    public void update() {
        setFinished();
    }

    @Override
    public String toString() {
        return "Intake note";
    }
    
}
