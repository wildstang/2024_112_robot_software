package org.wildstang.year2024.subsystems.notepath;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

public class notepath implements Subsystem{
    private WsSpark intakeMotor;
    private WsSpark feedMotor;
    private double intakeMotorSpeed = 0;
    private double feedMotorSpeed = 0;
    private DigitalInput aButton;

    @Override
    public void inputUpdate(Input source) {
        if (aButton.getValue()){
            feedMotorSpeed = 1;
            intakeMotorSpeed = 1;
        }
        else{
            feedMotorSpeed = 0;
            intakeMotorSpeed = 0;
        }

    }

    @Override
    public void init() {
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);
        feedMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.FEED);
        aButton = (DigitalInput) WsInputs.OPERATOR_FACE_DOWN.get();
        aButton.addInputListener(this);

    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        intakeMotor.setSpeed(intakeMotorSpeed);
        feedMotor.setSpeed(feedMotorSpeed);
    }

    @Override
    public void resetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetState'");
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }


}