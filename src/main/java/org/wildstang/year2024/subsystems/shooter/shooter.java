package org.wildstang.year2024.subsystems.shooter;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.hardware.roborio.outputs.config.WsSparkConfig;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;

public class shooter implements Subsystem {
    private WsSpark shooterMotor1;
    private WsSpark shooterMotor2;
    private double shooterMotor1Speed = 0.5;
    private double shooterMotor2Speed = 0.5;
    private AnalogInput leftTrigger;
    private DigitalInput dpadUp;
    private DigitalInput dpadDown;
    private boolean leftTriggerPressed = false;

    @Override
    public void inputUpdate(Input source) {
        if (leftTrigger.getValue() > 0.15){
            leftTriggerPressed = true;
        }
        else{
            leftTriggerPressed = false;
        }
        if (dpadUp.getValue()){
            shooterMotor1Speed += 0.05;
            shooterMotor2Speed += 0.05;
        }
        if (dpadDown.getValue()){
            shooterMotor1Speed -= 0.05;
            shooterMotor2Speed -= 0.05;
        }
    }

    @Override
    public void init() {
        shooterMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER1);
        shooterMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER2);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
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
        if (leftTriggerPressed){
            shooterMotor1.setSpeed(shooterMotor1Speed);
            shooterMotor2.setSpeed(shooterMotor2Speed);
        }
        //you can use else instead of this
        else{
            shooterMotor1.setSpeed(0);
            shooterMotor2.setSpeed(0);
        }

    }
    
    public void setShooterSpeed(double speed){
        shooterMotor1Speed = speed;
        shooterMotor2Speed = speed;
        leftTriggerPressed = true;
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
