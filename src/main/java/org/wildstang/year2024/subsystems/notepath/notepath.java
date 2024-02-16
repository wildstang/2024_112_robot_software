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
    private DigitalInput bButton;
    private DigitalInput beamBreakSensor;
    private DigitalInput rightTrigger;

    @Override
    public void inputUpdate(Input source) {
        //it wasn't listed originally, but it might be good to add another button that would run
        //the feed and intake backwards, in case it's needed for testing
        if (aButton.getValue() && beamBreakSensor.getValue() == false){
            feedMotorSpeed = 1;
            intakeMotorSpeed = 1;
        }
        else if (rightTrigger.getValue() && beamBreakSensor.getValue()){
            feedMotorSpeed = 1;
        }
        else if (bButton.getValue()){
            feedMotorSpeed = -1;
            intakeMotorSpeed = -1;
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
        bButton = (DigitalInput) WsInputs.OPERATOR_FACE_RIGHT.get();
        bButton.addInputListener(this);
        rightTrigger = (DigitalInput) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        rightTrigger.addInputListener(this);
        beamBreakSensor = (DigitalInput) WsInputs.BEAMBREAK_SENSOR.get();
        beamBreakSensor.addInputListener(this);

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