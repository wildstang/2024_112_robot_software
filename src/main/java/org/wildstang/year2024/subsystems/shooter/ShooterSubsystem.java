package org.wildstang.year2024.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;

import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.hardware.roborio.outputs.XboxControllerOutput;

import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;

import org.wildstang.year2024.subsystems.LED.LedSubsystem;
import org.wildstang.year2024.subsystems.LED.LedSubsystem.LedColor;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class  ShooterSubsystem implements Subsystem{

    public enum shooterType {INIT_SPEAKER, INIT_AMP, OUTTAKE, WAIT, FLOOR_INTAKE, STOW, SHOOT, SHOOTER_OFF, SHOOTER_EXIT_DELAY_1, SHOOTER_EXIT_DELAY_2, OVERRIDE, IDLE, HANG, SOURCE_INTAKE, SOURCE_STOW_1, SOURCE_STOW_2, STOW_REVERSE, SHOOTER_OFF_2};
    private shooterType shooterState;

    public WsSpark angleMotor;
    public WsSpark shooterMotor1, shooterMotor2;
    public WsSpark feedMotor;
    private WsSpark intakeMotor;
    private WsSpark hoodMotor;

    private DigitalInput leftBumper, rightBumper;
    private DigitalInput dpadRight, dpadLeft, dpadUp, dpadDown;
    private DigitalInput leftStickButton, rightStickButton;
    private DigitalInput start;
    private XboxControllerOutput xboxController;

    public AbsoluteEncoder pivotEncoder;
    private DigitalInput intakeBeamBreak, shooterBeamBreak;

    public SwerveDrive swerve;

    private boolean sensorOverride, sourceMode;
    private double distance;

    private boolean hood_deploy;
    private double hoodPos;
    private double hoodOutput;

    private boolean shooterEnable;
    private double goalVel, curVel, velErr;
    private double shooterOutput;

    private double goalPos, curPos, posErr;
    private double posOut;
    
    private double feedMotorOutput, intakeMotorOutput;

    private int iterCount;

    // public double[] speeds = {0.5, 0.6, 0.7, 0.8, 0.9, 1};

    // public double[] angles = {1,1,1,1,1,1};

    // public double[] distanceMarks = {1,1,1,1,1,1};

    // public int[] indexes = new int[2];

    Timer timer;

    @Override
    public void init() {
        /**** Button Inputs ****/
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        dpadRight.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);
        leftStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_BUTTON);
        leftStickButton.addInputListener(this);
        rightStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_BUTTON);
        rightStickButton.addInputListener(this);
        start = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_START);
        start.addInputListener(this);
       
        /**** Motors ****/
        shooterMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER1);
        shooterMotor1.setCoast();
        shooterMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER2);
        shooterMotor2.setCoast();

        angleMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER_ANGLE1);
        angleMotor.setBrake();

        hoodMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        hoodMotor.setBrake();

        feedMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.FEED);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);

        /**** Abs Encoders ****/
        pivotEncoder = angleMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(2 * Math.PI);

        /**** Beam Break Sensors ****/
        intakeBeamBreak = (DigitalInput) Core.getInputManager().getInput(WsInputs.BEAMBREAK_SENSOR_INTAKE);
        intakeBeamBreak.addInputListener(this);
        shooterBeamBreak = (DigitalInput) Core.getInputManager().getInput(WsInputs.BEAMBREAK_SENSOR_SHOOTER);
        shooterBeamBreak.addInputListener(this);

        /**** Other ****/
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        xboxController = (XboxControllerOutput) Core.getOutputManager().getOutput(WsOutputs.XBOXCONTROLLER);
        timer = new Timer();

        resetState();
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == leftStickButton && leftStickButton.getValue()) {
            sensorOverride = !sensorOverride;
            if (sensorOverride) {
                shooterState = shooterType.OVERRIDE;
            } else {
                shooterState = shooterType.WAIT;
            }
        }

        if (source == rightStickButton && rightStickButton.getValue()) {
            sourceMode = !sourceMode;
        }

        if (!sensorOverride) {
            if (source == leftBumper && leftBumper.getValue()){
                shooterState = shooterType.INIT_SPEAKER;
                Log.warn("INIT_SPEAKER");
            } else if (source == dpadUp && dpadUp.getValue()) {
                shooterState = shooterType.INIT_AMP;
                Log.warn("INIT_AMP");
            } else if (source == rightBumper){
                if (rightBumper.getValue()) {
                    if (sourceMode) {
                        shooterState = shooterType.SOURCE_INTAKE;
                        Log.warn("SOURCE_INTAKE");
                    } else {
                        shooterState = shooterType.FLOOR_INTAKE;
                        Log.warn("FLOOR_INTAKE");
                    }
                } 
            } else if (source == dpadDown){
                if(dpadDown.getValue()){
                    shooterState = shooterType.OUTTAKE;
                    Log.warn("OUTTAKE");
                } else {
                    shooterState = shooterType.WAIT;
                    Log.warn("WAIT");
                }
            } else if (source == start && start.getValue()) {
                shooterState = shooterType.HANG;
                Log.warn("HANG");
            }
        } else {
            if (source == leftBumper){
                if (leftBumper.getValue()) {
                    goalVel = ShooterConstants.SPEAKER_SPEED;
                    shooterEnable = true;
                } else {
                    shooterEnable = false;
                    goalPos = ArmConstants.SOFT_STOP_LOW;
                    feedMotorOutput = 0.0;
                    intakeMotorOutput = 0.0;
                }
            }
            if (source == dpadUp) {
                if (dpadUp.getValue()) {
                    hood_deploy = true;
                    goalPos = ArmConstants.AMP_POS;
                    goalVel = ShooterConstants.AMP_SPEED;
                    shooterEnable = true;
                } else {
                    hood_deploy = false;
                    goalPos = ArmConstants.SOFT_STOP_LOW;
                    shooterEnable = false;
                    feedMotorOutput = 0.0;
                    intakeMotorOutput = 0.0;
                }
            } else if (source == rightBumper){
                if (rightBumper.getValue()) {
                    if (sourceMode) {
                        shooterOutput = ShooterConstants.SOURCE_INTAKE_SPEED;
                        feedMotorOutput = FeedConstants.FEED_SOURCE_OUTPUT;
                        goalPos = ArmConstants.SOURCE_INTAKE_POS;
                        shooterEnable = true;
                    } else {
                        feedMotorOutput = FeedConstants.FEED_IN_OUTPUT;
                        intakeMotorOutput = FeedConstants.INTAKE_IN_OUTPUT;
                        goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                        goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                    }
                } else {
                    feedMotorOutput = 0.0;
                    intakeMotorOutput = 0.0;
                    shooterEnable = false;
                    if (sourceMode) goalPos = ArmConstants.SOFT_STOP_LOW;
                }
            } else if (source == dpadDown){
                if(dpadDown.getValue()) {
                    goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                    goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                    feedMotorOutput = FeedConstants.FEED_OUT_OUTPUT;
                    intakeMotorOutput = FeedConstants.INTAKE_OUT_OUTPUT;
                    goalVel = ShooterConstants.OUTTAKE_SPEED;
                    shooterEnable = true;
                } else {
                    feedMotorOutput = 0.0;
                    intakeMotorOutput = 0.0;
                    shooterEnable = false;
                }
            } else if (source == start && start.getValue()) {
                goalPos = ArmConstants.SOFT_STOP_LOW;
            } else if (source == dpadLeft && dpadLeft.getValue()) {
                goalPos -= 0.04;
            } else if (source == dpadRight && dpadRight.getValue()) {
                goalPos += 0.04;
            }
        }
    }


    @Override
    public void update() {
        curVel = getShooterVelocity();
        curPos = getPivotPosition();
        hoodPos = hoodMotor.getPosition();

        switch (shooterState) {
            case INIT_SPEAKER:
                distance = swerve.getDistanceFromSpeaker();
                goalVel = getTargetSpeed(distance);
                goalPos = getTargetAngle(distance);
                shooterEnable = true;
                if(pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget()){
                    shooterState = shooterType.SHOOT;
                    Log.warn("SHOOT");
                }
                break;
            case INIT_AMP:
                goalVel = ShooterConstants.AMP_SPEED;
                goalPos = ArmConstants.AMP_POS;
                shooterEnable = true;
                hood_deploy = true;
                if (pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget()) {
                    shooterState = shooterType.SHOOT;
                    Log.warn("SHOOT");
                }
                break;
            case SHOOT:
                feedMotorOutput = FeedConstants.FEED_OUTPUT;
                intakeMotorOutput = FeedConstants.INTAKE_IN_OUTPUT;
                if(!intakeBeamBreak.getValue() && !shooterBeamBreak.getValue()){
                    shooterState = shooterType.SHOOTER_EXIT_DELAY_1;
                    Log.warn("SHOOTER_EXIT_DELAY_1");
                }
                break;
            case SHOOTER_EXIT_DELAY_1:
                intakeMotorOutput = 0.0;
                iterCount += 1;
                if(iterCount > 30 && !shooterBeamBreak.getValue()){
                    iterCount = 0;
                    shooterState = shooterType.SHOOTER_OFF;
                    Log.warn("SHOOTER_OFF");
                }
                break;
            case SHOOTER_EXIT_DELAY_2:
                if(!shooterBeamBreak.getValue()){
                    shooterState = shooterType.SHOOTER_OFF;
                    Log.warn("SHOOTER_OFF");
                    timer.reset();
                    timer.start();
                }
                break;
            case SHOOTER_OFF:
                hood_deploy = false;
                shooterEnable = false;
                feedMotorOutput = 0.0;
                goalPos = ArmConstants.SOFT_STOP_LOW;
                LedSubsystem.ledState = LedColor.GREEN;
                if (pivotIsAtTarget()){
                    shooterState = shooterType.WAIT;
                    Log.warn("WAIT");
                }
                break;
            case SHOOTER_OFF_2:
                hood_deploy = false;
                shooterEnable = false;
                feedMotorOutput = 0.0;
                shooterState = shooterType.WAIT;
                Log.warn("WAIT");
                break;
            case FLOOR_INTAKE:
                goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                intakeMotorOutput = FeedConstants.INTAKE_IN_OUTPUT;
                if(intakeBeamBreak.getValue()){
                    LedSubsystem.ledState = LedColor.FLASH_ORANGE;
                    shooterState = shooterType.STOW;  // signifies that note is stowed
                    Log.warn("STOW");
                    timer.reset();
                    timer.start();
                }
                break;
            case SOURCE_INTAKE:
                goalPos = ArmConstants.SOURCE_INTAKE_POS;
                goalVel = ShooterConstants.SOURCE_INTAKE_SPEED;
                feedMotorOutput = FeedConstants.FEED_SOURCE_OUTPUT;
                if (intakeBeamBreak.getValue()) {
                    LedSubsystem.ledState = LedColor.FLASH_ORANGE;
                    shooterState = shooterType.SOURCE_STOW_1;
                    Log.warn("SOURCE_STOW_1");
                    timer.reset();
                    timer.start();
                }
                break;
            case SOURCE_STOW_1:
                if (shooterBeamBreak.getValue()){
                    shooterState = shooterType.SOURCE_STOW_2;
                    Log.warn("SOURCE_STOW_2");
                }
                break;
            case SOURCE_STOW_2:
                if (!shooterBeamBreak.getValue()){
                    shooterState = shooterType.STOW;
                    Log.warn("STOW");
                }
                break;
            case STOW:
                intakeMotorOutput = FeedConstants.INTAKE_STOW_OUTPUT;
                feedMotorOutput = FeedConstants.FEED_IN_OUTPUT;
                if(shooterBeamBreak.getValue()){
                    shooterState = shooterType.STOW_REVERSE;
                    iterCount = 0;
                    Log.warn("STOW_REVERSE");
                }
                break;
            case STOW_REVERSE:
                feedMotorOutput = -FeedConstants.FEED_IN_OUTPUT;
                iterCount += 1;
                if (iterCount > 10) {
                    iterCount = 0;
                    shooterState = shooterType.IDLE;
                    Log.warn("IDLE");
                }
                break;
            case IDLE:
                feedMotorOutput = 0.0;
                intakeMotorOutput = 0.0;
                goalVel = 150.0;
                shooterEnable = true;
                LedSubsystem.ledState = LedColor.PULSE_BLUE;
                break;
            case OUTTAKE:
                goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                goalVel = ShooterConstants.OUTTAKE_SPEED;
                feedMotorOutput = FeedConstants.FEED_OUT_OUTPUT;
                intakeMotorOutput = FeedConstants.INTAKE_OUT_OUTPUT;
                shooterEnable = true;
                break;
            case WAIT:
                hood_deploy = false;
                feedMotorOutput = 0.0;
                intakeMotorOutput = 0.0;
                goalVel = 0.0;
                shooterEnable = false;
                if (timer.hasElapsed(1.5)){
                    if (sourceMode) {
                        LedSubsystem.ledState = LedColor.YELLOW;
                    } else {
                        LedSubsystem.ledState = LedColor.BLUE;
                    }
                }
                break;
            case HANG:
                goalPos = ArmConstants.SOFT_STOP_LOW;
                break;
            case OVERRIDE:
                LedSubsystem.ledState = LedColor.BLUE;
                break;
        }

        // Shooter control system
        goalVel = Math.min(goalVel, ShooterConstants.MAX_VEL);
        goalVel = Math.max(goalVel, -ShooterConstants.MAX_VEL);
        velErr = goalVel - curVel;
        shooterOutput = goalVel * ShooterConstants.kF + velErr * ShooterConstants.kP;
        if (!shooterEnable) {
            shooterOutput = 0.0;
        }

        // Pivot control system
        goalPos = Math.min(goalPos, ArmConstants.SOFT_STOP_HIGH);  // don't command a position higher than the soft stop
        goalPos = Math.max(goalPos, ArmConstants.SOFT_STOP_LOW);  // don't command a position lower than the soft stop
        posErr = goalPos - curPos;
        posOut = posErr * ArmConstants.kP;
        if (curPos <= ArmConstants.SOFT_STOP_LOW ){
            posOut = Math.max(0, posOut);
        } else if (curPos >= ArmConstants.SOFT_STOP_HIGH){
            posOut = Math.min(0, posOut);
        }

        // Hood control system
        if (hood_deploy){  //  && hoodPos < HoodConstants.DEPLOY_POS  TODO: decide if we want to stall or brake the motor
            hoodOutput = HoodConstants.DEPLOY_OUTPUT;
        } else if (hoodPos > HoodConstants.RETRACT_POS){
            hoodOutput = HoodConstants.RETRACT_OUTPUT;
        } else {
            hoodOutput = 0.0;
        }

        if (sensorOverride  && shooterEnable && pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget()) {
            feedMotorOutput = FeedConstants.FEED_OUTPUT;
            intakeMotorOutput = FeedConstants.INTAKE_IN_OUTPUT;
        }

        intakeMotor.setSpeed(intakeMotorOutput);
        feedMotor.setSpeed(feedMotorOutput);
        hoodMotor.setSpeed(hoodOutput);
        angleMotor.setSpeed(posOut);
        shooterMotor1.setSpeed(shooterOutput);
        shooterMotor2.setSpeed(-shooterOutput);

        if(timer.hasElapsed(1.5)){
            xboxController.setValue(0);
            timer.stop();
        } else {
            xboxController.setValue(1.0);
        }

        // Hood values
        SmartDashboard.putNumber("Hood position", hoodPos);
        SmartDashboard.putNumber("Hood output", hoodOutput);
        SmartDashboard.putBoolean("Hood at target", hoodIsAtTarget());

        // Feed values
        SmartDashboard.putNumber("Feed output", feedMotorOutput);
        SmartDashboard.putNumber("Intake output", intakeMotorOutput);

        // Shooter values
        SmartDashboard.putNumber("Shooter goal", goalVel);
        SmartDashboard.putNumber("Shooter velocity", curVel);
        SmartDashboard.putNumber("Shooter output", shooterOutput);
        SmartDashboard.putBoolean("Shooter at target", shooterIsAtTarget());

        // Pivot values
        SmartDashboard.putNumber("Pivot goal", goalPos);
        SmartDashboard.putNumber("Pivot position", curPos);
        SmartDashboard.putNumber("Pivot Output", posOut);
        SmartDashboard.putBoolean("Pivot at target", pivotIsAtTarget());

        // Beam Break Sensors
        SmartDashboard.putBoolean("Intake bb", intakeBeamBreak.getValue());
        SmartDashboard.putBoolean("Shooter bb", shooterBeamBreak.getValue());

        // State values
        SmartDashboard.putString("Shooter state", shooterState.name());
        SmartDashboard.putBoolean("Sensor Override", sensorOverride);
        SmartDashboard.putBoolean("Source Mode", sourceMode);
        SmartDashboard.putNumber("iterCount", iterCount);
    }
    
    @Override
    public void resetState() {
        iterCount = 0;
        sensorOverride = false;
        sourceMode = false;
        shooterEnable = false;
        goalVel = 0.0;
        hoodOutput = 0.0;
        hood_deploy = false;
        curPos = getPivotPosition();
        curVel = getShooterVelocity();
        goalPos = curPos;
        goalVel = 0;
        posOut = 0;
        feedMotorOutput = 0.0;
        intakeMotorOutput = 0.0;
        shooterState = shooterType.WAIT;
        timer.reset();
        timer.start();
    }

    public double getPivotPosition(){
        if (sensorOverride){
            return ((angleMotor.getPosition() * 2 * Math.PI / ArmConstants.RATIO) + ArmConstants.SOFT_STOP_LOW) % (2 * Math.PI);
        } else {
            return (pivotEncoder.getPosition() + ArmConstants.SOFT_STOP_LOW) % (2 * Math.PI);
        }
    }

    public double getTargetSpeed(double distance){
        // for(int i = 0; i < distanceMarks.length; i++){
        //     if((distance >= distanceMarks[i]) && (distance >= distanceMarks[i+1])){
        //         indexes[0] = i;
        //         indexes[1] = i+1;
        //     }
        // }

        // return (double)((speeds[indexes[0]]+((speeds[indexes[1]] - speeds[indexes[0]]) * 
        //             ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])))));
        return 500.0;
    }

    public double getTargetAngle(double distance){
        //  return angles[indexes[0]] + (((angles[indexes[1]] - angles[indexes[0]])) 
        //         * ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])));
        // return Math.atan(FieldConstants.SPEAKER_Z/distance)
        return 35 * Math.PI / 180.0;
    }

    public void setShooterState(shooterType newState){
        shooterState = newState;
    }

    public double getShooterVelocity() {
        return shooterMotor1.getVelocity() * ShooterConstants.RATIO * 2 * Math.PI / 60.0;
    }

    public boolean pivotIsAtTarget() {
        return Math.abs(curPos - goalPos) < ArmConstants.POS_DB;
    }

    public Boolean hoodIsAtTarget(){
        return (!hood_deploy && hoodPos < HoodConstants.RETRACT_POS) || (hood_deploy && hoodPos > HoodConstants.DEPLOY_POS);
    }

    public Boolean shooterIsAtTarget(){
        // return curVel >= goalVel;
        return Math.abs(curVel) >= goalVel * 0.9;
    }
    @Override
    public String getName() {
        return "Shooter";
    }
    
    @Override
    public void selfTest() {
    }

    public boolean isOff() {
        return shooterState == shooterType.WAIT;
    }
}
