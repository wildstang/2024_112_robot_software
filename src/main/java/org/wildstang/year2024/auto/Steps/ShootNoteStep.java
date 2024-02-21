package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.notepath.notepath;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;


public class ShootNoteStep extends AutoStep {
    private boolean shootPossible;
    private ShooterSubsystem shooter;
    private notepath Notepath;
    private double robotDistance;
    private SwerveDrive drive;

    public ShootNoteStep(boolean shootPossible){
        this.shootPossible = shootPossible;
    }

    @Override
    public void initialize() {
       shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
       Notepath = (notepath) Core.getSubsystemManager().getSubsystem(WsSubsystems.NOTEPATH);
       drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {
        if(shootPossible){
            robotDistance = drive.getDistanceFromSpeaker();
            shooter.setShooterTarget(true,robotDistance);
            if (shooter.angleAtTarget() && shooter.velocityAtTarget()){
                Notepath.setNotepathSpeed(false, true);
            }
        }
    }

    @Override
    public String toString() {
        return "Shoot Note step";
    }
    
}
