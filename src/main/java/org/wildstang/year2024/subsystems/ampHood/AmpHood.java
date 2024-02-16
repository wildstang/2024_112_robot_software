package org.wildstang.year2024.subsystems.ampHood;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsInputs;

public class AmpHood implements Subsystem{
    private DigitalInput aButton;
    private WsSpark ampHoodMotor; 
    private double initialEncoderPosition;
    private double ampHoodSpeed;

    @Override
    public void init() {
        aButton = (DigitalInput) WsInputs.OPERATOR_FACE_DOWN.get();
        aButton.addInputListener(this);
        ampHoodMotor =  (WsSpark) WsOutputs.AMPHOOD.get();
        initialEncoderPosition = ampHoodMotor.getPosition();
        ampHoodSpeed = 0; 
    }

   

    @Override
    public void update() {
        ampHoodMotor.setSpeed(ampHoodSpeed);
    }

    @Override
    public void inputUpdate(Input source) {
       if (aButton.getValue()){
        ampHoodSpeed = 0.5;
       }
       else if (ampHoodMotor.getPosition() > initialEncoderPosition){
            ampHoodSpeed = -1;
       }
       else{
        ampHoodSpeed = 0;
       }
    }
    @Override

    public String getName() {
        //make sure to put a string name for the system here
        return "AmpHood";
    }



    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }



    @Override
    public void resetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetState'");
    }
}

