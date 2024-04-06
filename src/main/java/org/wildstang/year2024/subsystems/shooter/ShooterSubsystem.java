package org.wildstang.year2024.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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
import org.wildstang.year2024.subsystems.swerve.FieldConstants;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class  ShooterSubsystem implements Subsystem{

    public enum shooterType {FLOOR_INTAKE, SOURCE_INTAKE, STOW, STOW_REVERSE, IDLE, INIT_SPEAKER, INIT_AMP, SHOOT, SHOOTER_EXIT_DELAY_1, SHOOTER_OFF, OUTTAKE, WAIT, AMP_FAKE, AMP_FAKE_OFF};
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
    public LedSubsystem leds;

    private boolean sensorOverride, sourceMode;

    private boolean hood_deploy;
    private double hoodPos;
    private double hoodOutput;

    private boolean shooterEnable;
    private double goalVel, curVel, velErr;
    private double shooterOutput;

    private double goalPos, curPos, posErr;
    private double pivotAdjustment;
    private double posOut;
    
    private double feedMotorOutput, intakeMotorOutput;

    private int iterCount;

    Timer timer;
    Timer shootMoveTimer;

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
        shooterMotor2.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        shooterMotor2.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        shooterMotor2.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        angleMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTER_ANGLE1);
        angleMotor.setBrake();

        hoodMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        hoodMotor.setBrake();

        feedMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.FEED);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);

        /**** Abs Encoders ****/
        pivotEncoder = angleMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(2.0 * Math.PI);

        /**** Beam Break Sensors ****/
        intakeBeamBreak = (DigitalInput) Core.getInputManager().getInput(WsInputs.BEAMBREAK_SENSOR_INTAKE);
        intakeBeamBreak.addInputListener(this);
        shooterBeamBreak = (DigitalInput) Core.getInputManager().getInput(WsInputs.BEAMBREAK_SENSOR_SHOOTER);
        shooterBeamBreak.addInputListener(this);

        /**** Other ****/
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        leds = (LedSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.LEDS);
        xboxController = (XboxControllerOutput) Core.getOutputManager().getOutput(WsOutputs.XBOXCONTROLLER);
        timer = new Timer();
        
        resetState();
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == rightStickButton) {
            if(rightStickButton.getValue()){
                if (shooterState == shooterType.AMP_FAKE) shooterState = shooterType.AMP_FAKE_OFF;
                else shooterState = shooterType.AMP_FAKE;
            }
        } else if (source == leftBumper && leftBumper.getValue()){
            shooterState = shooterType.INIT_SPEAKER;
            Log.warn("INIT_SPEAKER");
        } else if (source == dpadUp && dpadUp.getValue()) {
            shooterState = shooterType.INIT_AMP;
            Log.warn("INIT_AMP");
        } else if (source == rightBumper && rightBumper.getValue()){
            if (leftBumper.getValue() || dpadUp.getValue()) {
                shooterState = shooterType.SHOOT;
                Log.warn("SHOOT");
            } else {
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
        } else if (source == dpadLeft && dpadLeft.getValue()) {
            if (sensorOverride) goalPos = ArmConstants.SUBWOOFER_POS - pivotAdjustment;
            else pivotAdjustment += 0.05;
        } else if (source == dpadRight && dpadRight.getValue()) {
            if (sensorOverride) goalPos = ArmConstants.PODIUM_POS - pivotAdjustment;
            else pivotAdjustment -= 0.05;
        }
    }

    @Override
    public void update() {
    
        sensorOverride = swerve.sensorOverride;
        
        curVel = getShooterVelocity();
        curPos = getPivotPosition();
        hoodPos = hoodMotor.getPosition();


        switch (shooterState) {
            case AMP_FAKE:
                goalPos = ArmConstants.AMP_POS;
                hood_deploy = true;
                break;
            case AMP_FAKE_OFF:
                goalPos = ArmConstants.SOFT_STOP_LOW;
                hood_deploy = false;
                shooterState = shooterType.WAIT;
                break;
            case INIT_SPEAKER:
                if (!sensorOverride){
                    goalPos = getSpeakerElevation(swerve.getDistanceFromSpeaker()) - pivotAdjustment;
                }
                hood_deploy = false;
                goalVel = ShooterConstants.SPEAKER_SPEED;
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
                feedMotorOutput = FeedConstants.FEED_SHOOT_OUTPUT;
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
                    leds.ledState = LedColor.GREEN;
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
                if (pivotIsAtTarget()){
                    shooterState = shooterType.WAIT;
                    Log.warn("WAIT");
                }
                break;
            case FLOOR_INTAKE:
                goalPos = Math.min(Math.max(curPos, ArmConstants.MIN_INTAKE_POS), ArmConstants.MAX_INTAKE_POS);
                intakeMotorOutput = FeedConstants.INTAKE_IN_OUTPUT;
                if(intakeBeamBreak.getValue()){
                    leds.ledState = LedColor.FLASH_ORANGE;
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
                shooterEnable = true;
                if (intakeBeamBreak.getValue()) {
                    leds.ledState = LedColor.FLASH_ORANGE;
                    goalPos = ArmConstants.SOFT_STOP_LOW;
                    shooterState = shooterType.STOW;
                    Log.warn("STOW");
                    timer.reset();
                    timer.start();
                }
                break;
            case STOW:
                intakeMotorOutput = FeedConstants.INTAKE_STOW_OUTPUT;
                feedMotorOutput = FeedConstants.FEED_IN_OUTPUT;
                if(shooterBeamBreak.getValue()){
                    shooterState = shooterType.STOW_REVERSE;
                    leds.ledState = LedColor.PULSE_BLUE;
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
            case IDLE://start spooling up shooter motor after intaking
                feedMotorOutput = 0.0;
                intakeMotorOutput = 0.0;
                goalVel = ShooterConstants.AMP_SPEED;
                shooterEnable = true;
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
                // goalPos = ArmConstants.SOFT_STOP_LOW;
                goalVel = 0.0;
                shooterEnable = false;
                if (timer.hasElapsed(1.0)){
                    if (sourceMode) {
                        leds.ledState = LedColor.YELLOW;
                    } else {
                        leds.ledState = LedColor.BLUE;
                    }
                }
                break;
        }

        // Shooter control system
        goalVel = Math.min(goalVel, ShooterConstants.MAX_VEL);
        goalVel = Math.max(goalVel, -ShooterConstants.MAX_VEL);
        velErr = goalVel - curVel;
        shooterOutput = goalVel * ShooterConstants.kF + velErr * ShooterConstants.kP;
        shooterOutput = Math.min(Math.max(shooterOutput,-1.0),1.0);
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
        if (hood_deploy){
            if (hoodIsAtTarget()) {
                hoodOutput = 1.0;
            } else {
                hoodOutput = HoodConstants.DEPLOY_OUTPUT;
            }
        } else {
            if (hoodIsAtTarget()) {
                hoodOutput = 0.0;
            } else {
                hoodOutput = HoodConstants.RETRACT_OUTPUT;
            }
        }

        intakeMotor.setSpeed(intakeMotorOutput);
        feedMotor.setSpeed(feedMotorOutput);
        hoodMotor.setSpeed(hoodOutput);
        angleMotor.setSpeed(posOut);
        shooterMotor1.setSpeed(shooterOutput);
        shooterMotor2.setSpeed(-shooterOutput);

        if(timer.hasElapsed(1.0)){
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
        SmartDashboard.putBoolean("Pivot at target", pivotIsAtTarget());

        // Shooter values
        SmartDashboard.putNumber("Shooter goal", goalVel);
        SmartDashboard.putNumber("Shooter velocity", curVel);
        SmartDashboard.putNumber("Shooter output", shooterOutput);
        SmartDashboard.putBoolean("Shooter at target", shooterIsAtTarget());

        // Pivot values
        SmartDashboard.putNumber("Pivot goal", goalPos);
        SmartDashboard.putNumber("Pivot position", curPos);
        SmartDashboard.putNumber("Pivot Output", posOut);
        
        // Beam Break Sensors
        SmartDashboard.putBoolean("Intake bb", intakeBeamBreak.getValue());
        SmartDashboard.putBoolean("Shooter bb", shooterBeamBreak.getValue());

        // State values
        SmartDashboard.putString("Shooter state", shooterState.name());
        SmartDashboard.putBoolean("Sensor Override", sensorOverride);
        SmartDashboard.putBoolean("Source Mode", sourceMode);
        SmartDashboard.putNumber("iterCount", iterCount);

        // Vision values
        SmartDashboard.putNumber("Speaker Elevation", getSpeakerElevation(swerve.getDistanceFromSpeaker()));

        SmartDashboard.putNumber("pivot internal encoder", angleMotor.getPosition());

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
        pivotAdjustment = 0.0;
        timer.reset();
        timer.start();
    }

    @Override
    public String getName() {
        return "Shooter";
    }
    
    @Override
    public void selfTest() {
    }

    public double getPivotPosition(){
        return ((angleMotor.getPosition() * 2.0 * Math.PI / ArmConstants.RATIO) + ArmConstants.SOFT_STOP_LOW + 2.0 * Math.PI) % (2.0 * Math.PI);
        // return (pivotEncoder.getPosition() + ArmConstants.SOFT_STOP_LOW + pivotAdjustment) % (2 * Math.PI);
    }

    public double getSpeakerElevation(double distance){
        return Math.atan(FieldConstants.SPEAKER_Z/(distance+.235))+distance*.02; //+.08;
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
        return Math.abs(curVel - goalVel) <= ShooterConstants.VEL_DB;
    }

    public boolean isOff() {
        return shooterState == shooterType.WAIT;
    }

    public boolean isIdle() {
        return shooterState == shooterType.IDLE;
    }
}
