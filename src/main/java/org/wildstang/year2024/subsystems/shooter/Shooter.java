package org.wildstang.year2024.subsystems.shooter;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Subsystem {
    private WsSpark shooterMotor;
    private double goalVel, curVel, velErr;
    private double out;
    private Boolean shooterEnable;
    private DigitalInput leftBumper, rightBumper, dpadUp;

    @Override
    public void inputUpdate(Input source) {
        if (source == dpadUp && dpadUp.getValue()) {
            goalVel = 400;
            shooterEnable = true;
        } else if (source == leftBumper && leftBumper.getValue()){
            goalVel = 445;  // 445
            shooterEnable = true;
        } else if (source == rightBumper && rightBumper.getValue()) {
            goalVel = -300;
            shooterEnable = false;
        } else {
            shooterEnable = false;
        }
    }

    @Override
    public void init() {
        shooterMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER1);
        shooterMotor.setCoast();
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (shooterEnable) {
            goalVel = Math.min (goalVel, ShooterConstants.MAX_VEL);
            goalVel = Math.max(goalVel, -ShooterConstants.MAX_VEL);

            curVel = getVelocity();
            velErr = goalVel - curVel;

            out = goalVel * ShooterConstants.kF + velErr * ShooterConstants.kP;
            
            shooterMotor.setSpeed(out);
        } else {
            shooterMotor.setSpeed(0);
        }

        SmartDashboard.putNumber("shooter target", goalVel);
        SmartDashboard.putNumber("shooter speed", out);
        SmartDashboard.putBoolean("shooter at target", isAtTarget());
        SmartDashboard.putNumber("shooter velocity", getVelocity());
    }
    
    public void setShooterSpeed(double speed){
        goalVel = speed;
        shooterEnable = true;
    }

    public Boolean isAtTarget(){
        return Math.abs(goalVel - getVelocity()) < 10;
    }

    public double getVelocity() {
        return shooterMotor.getVelocity() * ShooterConstants.RATIO * 2 * Math.PI / 60.0;
    }

    @Override
    public void resetState() {
        shooterEnable = false;
        goalVel = 0.0;
    }

    @Override
    public String getName() {
        //make sure to include a name in this file
        return "Shooter";
    }
    
}
