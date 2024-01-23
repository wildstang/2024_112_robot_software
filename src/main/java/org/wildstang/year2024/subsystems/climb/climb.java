package org.wildstang.year2024.subsystems.climb;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.core.Subsystems;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.hardware.roborio.outputs.config.WsSparkConfig;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

public class Climb implements Subsystem{
    private WsSpark climbMotor;
    private AnalogInput joyStick;
    private double climbMotorSpeed;

     @Override
    public void inputUpdate(Input source) {
        if (joyStick.getValue() > 0.2){
            climbMotorSpeed = 1;
        }
        else if (joyStick.getValue() < -0.2){
            climbMotorSpeed = -1;
        }
        else{
            climbMotorSpeed = 0;

        }

    }

    @Override
    public void init() {
        climbMotor = (WsSpark) WsOutputs.CLIMB.get();
        joyStick = (AnalogInput) WsInputs.DRIVER_LEFT_JOYSTICK_Y.get();
        joyStick.addInputListener(this);

    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        climbMotor.setSpeed(climbMotorSpeed);
    }

    @Override
    public void resetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetState'");
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Climb";
    }

    

}