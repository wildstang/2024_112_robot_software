package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;
public class scoreAmp extends AutoStep {
    private boolean scoreAmpPossible;
    private ShooterSubsystem shooter;
    private double ampAngle;
    private double robotAmpDistance;
    private SwerveDrive swerve;

    public scoreAmp(boolean scoreAmpPossible){
        this.scoreAmpPossible = scoreAmpPossible; 
    }

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {
       if (scoreAmpPossible){
            shooter.setAngle(true, ampAngle);
            shooter.setRetract(false);
            robotAmpDistance = swerve.getDistanceFromAmp();
            shooter.setShooterSpeed(true, robotAmpDistance);
            if(shooter.angleAtTarget() && shooter.velocityAtTarget()){
                shooter.setNotepathSpeed(false, true);
            }
       };
    }

    @Override
    public String toString() {
        return "Score amp";
    }
    
}
