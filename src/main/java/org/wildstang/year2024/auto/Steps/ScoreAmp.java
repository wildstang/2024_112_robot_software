package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;

public class ScoreAmp extends AutoStep {
    private ShooterSubsystem shooter;

    public ScoreAmp(){
    }

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
    }

    @Override
    public void update() {
        shooter.setShooterState(shooterType.AMP);
        if (shooter.isOff()){
            setFinished();
        }
    }

    @Override
    public String toString() {
        return "Score amp";
    }
    
}
