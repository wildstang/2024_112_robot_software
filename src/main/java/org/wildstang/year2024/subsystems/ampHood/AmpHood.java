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
    private double initialEncoderPosition;
    private final double ampHoodSpeed = 0.25;
    private boolean retract;

    @Override
    public void init() {
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        ampHoodMotor =  (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        ampHoodMotor.setBrake();
        initialEncoderPosition = ampHoodMotor.getPosition();
        retract = true;
    }

   

    @Override
    public void update() {
        if (!retract && ampHoodMotor.getPosition() > -55){
            ampHoodMotor.setSpeed(-ampHoodSpeed);
        } 
       else if (ampHoodMotor.getPosition() < -1){
            ampHoodMotor.setSpeed(ampHoodSpeed);
       } else {
        ampHoodMotor.setSpeed(0);
       }
       SmartDashboard.putBoolean("hood", retract);
       SmartDashboard.putNumber("hood position", ampHoodMotor.getPosition());
    }

    @Override
    public void inputUpdate(Input source) {
       if (dpadUp.getValue()){
        retract = false;
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



    @Override
    public void selfTest() {
    }



    @Override
    public void resetState() {
    }
}

