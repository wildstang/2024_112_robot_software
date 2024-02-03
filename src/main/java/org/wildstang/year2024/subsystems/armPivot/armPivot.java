package org.wildstang.year2024.subsystems.armPivot;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;

public class armPivot implements Subsystem {
    /* 1. rename this ^ to be whatever your file is called
     * 2. go to getName() and change the string "template" to a string that describes this subsystem
     
     * 3. add any variables you need under this comment block (inputs, outputs, doubles/booleans/ints)
     * 4. Initialize any inputs and outputs under init()
     * 5. add your code where it should be in inputUpdate() and update()
     */

     
    private AnalogInput joystick;
    private WsSparkMax motor;
    private AbsoluteEncoder absEncoder;
    int direction = 0;
    double speed;
    public Wrist(WsSparkMax outputMotor){
        motor = outputMotor;
        motor.setBrake();
        absEncoder = motor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoder.setPositionConversionFactor(360.0);
        absEncoder.setVelocityConversionFactor(360.0/60.0);
        absEncoder.setInverted(SuperConts.TRUE_ENCODER_DIRECTION);
        absEncoder.setZeroOffset(253.11);
        motor.initClosedLoop(SuperConts.WRIST_P, SuperConts.WRIST_I, SuperConts.WRIST_D, 0, absEncoder, false);
        motor.setCurrentLimit(15, 15, 0);
    }
    public double getPosition(){
        return (absEncoder.getPosition())%360;
    }
    public void setPosition(double position){
        motor.setPosition((position)%360);
    }    
    @Override
    public void inputUpdate(Input source) {
        if(joystick.getValue() > 0 ){
            speed = -1.0;
        } else if(joystick.getValue() < 0){
            speed = 1.0;
        } 
        
    }

    @Override
    public void init() {
        joystick = (AnalogInput) WsInputs.DRIVER_RIGHT_JOYSTICK_Y_DOWN.get();
        joystick.addInputListener(this);
        motor = (WsSpark) WsOutputs.ANGLE1.get();
    }

    @Override
    public void update() {
        motor.setSpeed(speed);
       
    }

    @Override
    public String getName() {
        return "Arm Pivot";
    }
    @Override
    public void resetState() {
        speed = 0;
    }
    @Override
    public void selfTest() {
    }
}
