package org.wildstang.year2024.subsystems.notepath;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Notepath implements Subsystem{
    private WsSpark intakeMotor;
    private WsSpark feedMotor;
    private double intakeMotorSpeed = 0;
    private double feedMotorSpeed = 0;
    private DigitalInput rightBumper;
    private DigitalInput dpadDown;

    @Override
    public void inputUpdate(Input source) {
        //it wasn't listed originally, but it might be good to add another button that would run
        //the feed and intake backwards, in case it's needed for testing
        if (rightBumper.getValue()){
            feedMotorSpeed = -0.25;
            intakeMotorSpeed = 0.8;
        }
        else if (dpadDown.getValue()){
            feedMotorSpeed = 0.25;
            intakeMotorSpeed = -0.4;
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
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);

    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        intakeMotor.setSpeed(intakeMotorSpeed);
        feedMotor.setSpeed(feedMotorSpeed);
        SmartDashboard.putNumber("intake speed", intakeMotorSpeed);
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