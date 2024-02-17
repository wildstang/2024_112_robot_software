package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.notepath.notepath;

public class intakeNoteStep extends AutoStep{
    private boolean speedForward;
    private notepath notepath;
    private boolean speedBackwards;

    public intakeNoteStep(boolean speedForward, boolean speedBackwards){
        this.speedForward = speedForward;
        this.speedBackwards = speedBackwards;
    }

    @Override
    public void initialize() {
        notepath = (notepath) Core.getSubsystemManager().getSubsystem(WsSubsystems.NOTEPATH);
    }

    @Override
    public void update() {
        if (speedForward && speedBackwards){
            notepath.setNotepathSpeed(true, true);
            //moves the feed and intake motor speed to 1
        }
        else if( speedForward == false && speedForward == false) {
            //Sets the feed and intake motor speeds to -1
            notepath.setNotepathSpeed(false, false);
        }
        else{
            //Sets the feed and motor speeds to 0
            notepath.setNotepathSpeed(true,false);
        }
        setFinished();
    }

    @Override
    public String toString() {
        return "Intake note";
    }
    
}
