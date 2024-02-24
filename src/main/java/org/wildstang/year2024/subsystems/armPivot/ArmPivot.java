package org.wildstang.year2024.subsystems.armPivot;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

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

    private DigitalInput dpadRight, dpadLeft;
    private DigitalInput dpadUp, rightBumper, leftBumper;
    private WsSpark angleMotor;
    private AbsoluteEncoder absEncoder;
    double goalPos, curPos, curPosErr;
    double goalVel, curVel, curVelErr;
    double curOut = 0;

    @Override
    public void inputUpdate(Input source) {
        if (source == dpadUp){
            if (dpadUp.getValue()) {
                goalPos = 100 * Math.PI / 180.0;  // TODO: verify amp scoring angle
            } else {
                goalPos = 35 * Math.PI / 180.0;  // TODO: verify podium angle
            }
        } else if (source == rightBumper) {
            if (rightBumper.getValue()) {
                goalPos = 100 * Math.PI / 180.0;  // TODO: verify intake angle
            } else {
                goalPos = 35 * Math.PI / 180.0;  // TODO: verify podium angle
            }
        } else if (source == dpadRight && dpadRight.getValue()){
            goalPos += 5 * Math.PI / 180.0;
        } else if (source == dpadLeft && dpadLeft.getValue()){
            goalPos -= 5 * Math.PI / 180.0;
        } else if (source == leftBumper && !leftBumper.getValue()){
            goalPos = 35 * Math.PI / 180.0;  // TODO: verify podium angle
        }
    }

    @Override
    public void init() {
        dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        dpadRight.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        angleMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER_ANGLE1);
        angleMotor.setBrake();
        absEncoder = angleMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        // absEncoder.setPositionConversionFactor(360.0);
        // absEncoder.setVelocityConversionFactor(360.0 / 60.0);
        curPos = getPosition();
    }

    @Override
    public void update() {
        curPos = getPosition();
        // curVel = absEncoder.getVelocity() * Math.PI / 180.0;
        goalPos = Math.min(goalPos, ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0);  // don't command a position higher than the soft stop
        goalPos = Math.max(goalPos, ArmConstants.ZERO_OFFSET * Math.PI / 180.0);  // don't command a position lower than the soft stop
        curPosErr = goalPos - curPos;

        // goalVel = getVelocityTarget(goalVel, curPosErr);
        // curVelErr = goalVel - curVel;
        // curOut = goalVel * ArmConstants.kF + curVelErr * ArmConstants.VEL_P;

        curOut = curPosErr * 0.8;

        // if(isAtTarget()){
        //     curOut = 0;
        // }

        if (curPos <= ArmConstants.ZERO_OFFSET * Math.PI / 180.0 ){
            curOut = Math.max(0, curOut);
        } else if (curPos >= ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0){
            curOut = Math.min(0, curOut);
        }

        angleMotor.setSpeed(curOut);

        SmartDashboard.putNumber("goal position", goalPos);
        SmartDashboard.putNumber("shooter position", getPosition());
        SmartDashboard.putNumber("shooter curPosError", curPosErr);
        SmartDashboard.putNumber("shooter goal velocity", goalVel);
        SmartDashboard.putNumber("shooter velocity error", curVelErr);
        SmartDashboard.putNumber("Output", curOut);
        SmartDashboard.putBoolean("pivot at target", isAtTarget());
    }

    public double getVelocityTarget(double curVel, double curPosErr){
        if (curPosErr > 0){
            if (curPosErr <= curVel * Math.abs(curVel / ArmConstants.MAX_ACC) * .5){
                return curVel - (ArmConstants.MAX_ACC * ArmConstants.DELTA_T);
            } else if (curVel < ArmConstants.MAX_VEL){
                return Math.min(curVel + ArmConstants.MAX_ACC * ArmConstants.DELTA_T, ArmConstants.MAX_VEL);
            } else{
                return ArmConstants.MAX_VEL;
            }
        } else {
            if (curPosErr >= curVel * Math.abs(curVel / ArmConstants.MAX_ACC) * .5){
                return curVel + (ArmConstants.MAX_ACC * ArmConstants.DELTA_T);
            } else if (curVel > -ArmConstants.MAX_VEL){
                return Math.max(curVel - ArmConstants.MAX_ACC * ArmConstants.DELTA_T, -ArmConstants.MAX_VEL);
            } else{
                return -ArmConstants.MAX_VEL;
            }
        }
    }

    public boolean isAtTarget() {
        return Math.abs(curPosErr) < ArmConstants.POS_DB && Math.abs(curVel) < ArmConstants.VEL_DB;
    }

    @Override
    public String getName() {
        return "Arm Pivot";
    }

    @Override
    public void resetState() {
        curPos = getPosition();
        curVel = absEncoder.getVelocity();
        goalPos = curPos;
        goalVel = 0;
        curOut = 0;
    }

    @Override
    public void selfTest() {
    }

    public double getPosition(){
        // return ((absEncoder.getPosition() + ArmConstants.ZERO_OFFSET) % 360) * Math.PI / 180.0;
        return (angleMotor.getPosition() * 2 * Math.PI / ArmConstants.RATIO) + ArmConstants.ZERO_OFFSET * Math.PI / 180.0;
    }
    
    public void setPosition(double position){
        goalPos = position;
    }

}
