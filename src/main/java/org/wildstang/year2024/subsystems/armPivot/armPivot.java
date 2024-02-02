package org.wildstang.year2024.subsystems.armPivot;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class armPivot implements Subsystem{
    /* 1. rename this ^ to be whatever your file is called
     * 2. go to getName() and change the string "template" to a string that describes this subsystem
     
     * 3. add any variables you need under this comment block (inputs, outputs, doubles/booleans/ints)
     * 4. Initialize any inputs and outputs under init()
     * 5. add your code where it should be in inputUpdate() and update()
     */

     //example
    //  private DigitalInput button;
    //  private WsSpark motor;
    
    private WsSpark armPivotMotor1;
    private WsSpark armPivotMotor2;
    private AbsoluteEncoder absEncoder;
    private DigitalInput xButton;
    private DigitalInput yButton;
    private AnalogInput joyStickY;
    private double armPosition;

    public double getPosition(){
        return (absEncoder.getPosition())%360;
    }
    public void setPosition(double position){
        armPivotMotor1.setPosition((position)%360);
        armPivotMotor2.setPosition((position)%360);
    }    

    @Override
    public void inputUpdate(Input source) {
        if (xButton.getValue()){
            armPosition = 0;
        }
        else if (yButton.getValue()){
            armPosition = 180;
        }
        else if (joyStickY.getValue() > 0.05){
            armPosition += 1;
        }
        else if (joyStickY.getValue() < -0.05){
            armPosition -= 1;
        }
    }


    @Override
    public void init() {
        armPivotMotor1 = (WsSpark) WsOutputs.ARMPIVOT1.get();
        armPivotMotor2 = (WsSpark) WsOutputs.ARMPIVOT2.get();
        
        armPivotMotor1.setBrake();
        absEncoder = armPivotMotor1.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoder.setPositionConversionFactor(360.0);
        absEncoder.setVelocityConversionFactor(360.0/60.0);
        absEncoder.setInverted(true);
        absEncoder.setZeroOffset(253.11);//310
        armPivotMotor1.initClosedLoop(1, 0, 0, 0, absEncoder, false);
        armPivotMotor1.setCurrentLimit(15, 15, 0);
        xButton = (DigitalInput) WsInputs.OPERATOR_FACE_LEFT.get();
        xButton.addInputListener(this);
        yButton = (DigitalInput) WsInputs.OPERATOR_FACE_UP.get();
        yButton.addInputListener(this);
        joyStickY = (AnalogInput) WsInputs.DRIVER_RIGHT_JOYSTICK_Y.get();
        joyStickY.addInputListener(this);

    }

    @Override
    public void update() {
        setPosition(armPosition);
    }

    @Override
    public String getName() {
        //
        return "Arm Pivot";
    }
    @Override
    public void resetState() {
    }
    @Override
    public void selfTest() {
    }
}
