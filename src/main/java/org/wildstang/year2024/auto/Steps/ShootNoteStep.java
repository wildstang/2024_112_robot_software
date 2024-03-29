package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;

public class ShootNoteStep extends AutoStep {
    private ShooterSubsystem shooter;
    private SwerveDrive drive;
    private Timer timer;
    private double shootTime;
    boolean hasShot;

    public ShootNoteStep(double forceShootTime){
        timer = new Timer();
        shootTime = forceShootTime;
    }
    public ShootNoteStep(){
        timer = new Timer();
        shootTime = 3.0;
        hasShot = false;
    }

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        shooter.setShooterState(shooterType.INIT_SPEAKER);
        drive.autoNoteAim(false);
        drive.aimAtSpeaker(true);
        Log.warn("Shoot Step");
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        if (shooter.isOff() || timer.hasElapsed(shootTime+.25)){
            drive.aimAtSpeaker(false);
            timer.stop();
            setFinished();
        } else if (timer.hasElapsed(shootTime)){
            if(!hasShot) {
                shooter.setShooterState(shooterType.SHOOT);
                hasShot = true;
            }
        }
        
    }

    @Override
    public String toString() {
        return "Shoot Note step";
    }
    
}
