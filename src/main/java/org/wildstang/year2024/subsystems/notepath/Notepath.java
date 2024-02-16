package org.wildstang.year2024.subsystems.notepath;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

public class Notepath implements Subsystem{
    private WsSpark intakeMotor;
    private WsSpark feedMotor;
    private double intakeMotorSpeed = 0;
    private double feedMotorSpeed = 0;
    private DigitalInput aButton;
    private DigitalInput bButton;

    @Override
    public void inputUpdate(Input source) {
        //it wasn't listed originally, but it might be good to add another button that would run
        //the feed and intake backwards, in case it's needed for testing
        if (aButton.getValue()){
            feedMotorSpeed = -0.25;
            intakeMotorSpeed = 0.5;
        }
        else if (bButton.getValue()){
            feedMotorSpeed = 0.25;
            intakeMotorSpeed = -0.5;
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
        aButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        aButton.addInputListener(this);
        bButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        bButton.addInputListener(this);

    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        intakeMotor.setSpeed(intakeMotorSpeed);
        feedMotor.setSpeed(feedMotorSpeed);
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        //make sure to put a string name for the system here
        return "notepath";
    }


}