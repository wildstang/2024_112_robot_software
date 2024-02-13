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
    private WsSpark shooterMotor1;
    private WsSpark shooterMotor2;
    private double shooterSpeed = 0.5;
    private DigitalInput leftBumper;
    private DigitalInput dpadUp;
    private DigitalInput dpadDown;
    private boolean leftBumperPressed = false;

    @Override
    public void inputUpdate(Input source) {
        if (source == leftBumper && leftBumper.getValue()){
            leftBumperPressed = true;
        }
        else{
            leftBumperPressed = false;
        }
        if (source == dpadUp && dpadUp.getValue()){
            shooterSpeed += 0.05;
        }
        if (source == dpadDown && dpadDown.getValue()){
            shooterSpeed -= 0.05;
        }
    }

    @Override
    public void init() {
        shooterMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER1);
        shooterMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER2);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        //remember to add .addInputListener(this) for all inputs (including dpadUp and dpadDown)
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);


    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (leftBumperPressed){
            shooterMotor1.setSpeed(shooterSpeed);
            shooterMotor2.setSpeed(shooterSpeed);
        }
        //you can use else instead of this
        else{
            shooterMotor1.setSpeed(0);
            shooterMotor2.setSpeed(0);
        }
        SmartDashboard.putNumber("shooter speed", shooterSpeed);

    }
    
    public void setShooterSpeed(double speed){
        shooterSpeed = speed;
        leftBumperPressed = true;
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        //make sure to include a name in this file
        return "shooter";
    }
    
}
