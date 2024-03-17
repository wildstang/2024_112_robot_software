package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ArmConstants;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem.shooterType;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class ShootNoteStep extends AutoStep {
    private ShooterSubsystem shooter;
    private SwerveDrive drive;

    public ShootNoteStep(){
    }

    @Override
    public void initialize() {
       shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
       drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        shooter.setGoalPos(ArmConstants.SUBWOOFER_POS);
        shooter.setShooterState(shooterType.INIT_SPEAKER);
    }

    @Override
    public void update() {
        // drive.setToSpeaker();
        if (shooter.isOff()){
            // drive.setToAuto();
            setFinished();
        }
        
    }

    @Override
    public String toString() {
        return "Shoot Note step";
    }
    
}
