package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.shooter.ShooterSubsystem;

public class intakeNoteStep extends AutoStep{
    private boolean speedForward;
    private ShooterSubsystem shooter;
    private boolean speedBackwards;

    public intakeNoteStep(boolean speedForward, boolean speedBackwards){
        this.speedForward = speedForward;
        this.speedBackwards = speedBackwards;
    }

    @Override
    public void initialize() {
        shooter = (ShooterSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
    }

    @Override
    public void update() {
        if (speedForward && speedBackwards){
            shooter.setNotepathSpeed(true, true);
            //moves the feed and intake motor speed to 1
        }
        else if( speedForward == false && speedForward == false) {
            //Sets the feed and intake motor speeds to -1
            shooter.setNotepathSpeed(false, false);
        }
        else{
            //Sets the feed and motor speeds to 0
            shooter.setNotepathSpeed(false,true);
        }
        setFinished();
    }

    @Override
    public String toString() {
        return "Intake note";
    }
    
}
