package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;

public class IntakeNoteStep extends AutoStep{
    private ShooterSubsystem shooter;

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        shooter.setShooterState(shooterType.FLOOR_INTAKE);
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
