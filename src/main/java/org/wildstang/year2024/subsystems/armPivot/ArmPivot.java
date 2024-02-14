package org.wildstang.year2024.subsystems.armPivot;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAbsoluteEncoder;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPivot implements Subsystem {
    /* 1. rename this ^ to be whatever your file is called
     * 2. go to getName() and change the string "template" to a string that describes this subsystem
     
     * 3. add any variables you need under this comment block (inputs, outputs, doubles/booleans/ints)
     * 4. Initialize any inputs and outputs under init()
     * 5. add your code where it should be in inputUpdate() and update()
     */

    private DigitalInput angleUp;
    private DigitalInput angleDown;
    private WsSpark angleMotor1;
    private WsSpark angleMotor2;
    private AbsoluteEncoder absEncoder;
    double goalPos = 0;
    double error = 0;

    @Override
    public void inputUpdate(Input source) {
        if (source == angleUp && angleUp.getValue()){
            goalPos += 5;
        }
        if (source == angleDown && angleDown.getValue()){
            goalPos += 5;
        }
    }

    @Override
    public void init() {
        angleUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.OPERATOR_DPAD_UP);
        angleUp.addInputListener(this);
        angleDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.OPERATOR_DPAD_DOWN);
        angleDown.addInputListener(this);
        angleMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER_ANGLE1);
        angleMotor1.setBrake();
        angleMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER_ANGLE2);
        angleMotor2.setBrake();
        absEncoder = angleMotor1.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoder.setPositionConversionFactor(360.0);
        absEncoder.setVelocityConversionFactor(360.0/60.0);
        // absEncoder.setInverted(true);
        absEncoder.setZeroOffset(ArmConstants.ZERO_OFFSET);
        // angleMotor1.initClosedLoop(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, 0, absEncoder, false);
    }

    @Override
    public void update() {
        // angleMotor1.setPosition(goalPos);
        // angleMotor2.setPosition(goalPos);
        error = goalPos - getPosition();
        angleMotor1.setSpeed(error*.1);
        angleMotor2.setSpeed(error*.1);

        SmartDashboard.putNumber("goal position", goalPos);
        SmartDashboard.putNumber("shooter position", getPosition());
        SmartDashboard.putNumber("shooter error", error);
    }

    @Override
    public String getName() {
        return "Arm Pivot";
    }

    @Override
    public void resetState() {
        goalPos = getPosition();
    }

    @Override
    public void selfTest() {
    }

    public double getPosition(){
        return (absEncoder.getPosition())%360;
    }
    
    public void setPosition(double position){
        goalPos = position;
    }

}
