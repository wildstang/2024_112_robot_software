package org.wildstang.year2024.subsystems.ampHood;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmpHood implements Subsystem{
    private DigitalInput dpadUp;
    private WsSpark ampHoodMotor; 
    private double ampHoodSpeed;
    private boolean retract;

    @Override
    public void init() {
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        ampHoodMotor =  (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        ampHoodMotor.setBrake();
        resetState();
    }

   

    @Override
    public void update() {
        if (!retract && ampHoodMotor.getPosition() < 1.75){
            ampHoodSpeed = 0.15;
        } else if (ampHoodMotor.getPosition() > 0.1){
            ampHoodSpeed = -0.15;
        } else {
            ampHoodSpeed = 0.0;
        }
       ampHoodMotor.setSpeed(ampHoodSpeed);

       SmartDashboard.putNumber("hood position", ampHoodMotor.getPosition());
       SmartDashboard.putNumber("amp speed", ampHoodSpeed);
       SmartDashboard.putBoolean("hood at target", isAtTarget());
    }

    @Override
    public void inputUpdate(Input source) {
       if (dpadUp.getValue()){
        retract = true;
       }
       else{
        retract = true;
       }
    }
    @Override

    public String getName() {
        //make sure to put a string name for the system here
        return "AmpHood";
    }

    public Boolean isAtTarget(){
        return (retract && ampHoodMotor.getPosition() < 0.1) || (!retract && ampHoodMotor.getPosition() > 1.9);  // TODO: Check position travel limits
    }



    @Override
    public void selfTest() {
    }



    @Override
    public void resetState() {
        ampHoodSpeed = 0.0;
        retract = true;
    }
}

