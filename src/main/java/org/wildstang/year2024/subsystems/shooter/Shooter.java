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
    private double shooterSpeed = 0.45;
    private DigitalInput leftBumper;
    // private DigitalInput dpadRight;
    // private DigitalInput dpadLeft;
    private boolean shooterEnable = false;

    @Override
    public void inputUpdate(Input source) {
        if (source == leftBumper && leftBumper.getValue()){
            shooterEnable = true;
        }
        else{
            shooterEnable = false;
        }
        // if (source == dpadRight && dpadRight.getValue()){
        //     shooterSpeed += 0.05;
        // }
        // if (source == dpadLeft && dpadLeft.getValue()){
        //     shooterSpeed -= 0.05;
        // }
    }

    @Override
    public void init() {
        shooterMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER1);
        shooterMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER2);

        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        // dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        // dpadRight.addInputListener(this);
        // dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        // dpadLeft.addInputListener(this);

    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (shooterEnable){
            shooterMotor1.setSpeed(shooterSpeed);
            shooterMotor2.setSpeed(-shooterSpeed);
        }
        else{
            shooterMotor1.setSpeed(0);
            shooterMotor2.setSpeed(0);
        }
        SmartDashboard.putNumber("shooter speed", shooterSpeed);

    }
    
    public void setShooterSpeed(double speed){
        shooterSpeed = speed;
        shooterEnable = true;
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
