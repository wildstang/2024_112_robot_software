package org.wildstang.year2024.subsystems.shooter;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class  ShooterSubsystem implements Subsystem{

    public enum shooterType {SPEAKER, AMP, INTAKE, OUTTAKE, OFF};
    private shooterType shooterState;

    public WsSpark angleMotor;
    public WsSpark shooterMotor;
    public WsSpark feedMotor;
    private WsSpark intakeMotor;
    private WsSpark hoodMotor;
    
    public DigitalInput leftTrigger;
    private DigitalInput dpadRight;
    private DigitalInput dpadLeft, dpadDown;
    private DigitalInput beamBreakSensor;
    private DigitalInput aButton;
    private DigitalInput bButton;
    private DigitalInput leftBumper, rightBumper, dpadUp;
    
    public AbsoluteEncoder absEncoderAngle;

    public SwerveDrive swerve;

    public double robot_Distance;
    public double motorSpeed;
    public double motorAngle;
    private double velocityDifference;
    private double angleDifference;
    private double feedMotorOutput;
    private double intakeMotorOutput;
    private boolean retract;
    private double goalVel, curVel, velErr;
    private double shooterOut;
    private Boolean shooterEnable;
    private double goalPos, curPos, curPosErr;
    private double curVelErr;
    private double posOut;
    private double hoodOutput;

    public double[] speeds = {0.5, 0.6, 0.7, 0.8, 0.9, 1};

    public double[] angles = {1,1,1,1,1,1};

    public double[] distanceMarks = {1,1,1,1,1,1};

    public int[] indexes = new int[2];

    @Override
    public void init() {
        /**** Abs Encoders ****/
        absEncoderAngle = angleMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        
        /**** Button Inputs ****/
        leftTrigger = (DigitalInput) WsInputs.DRIVER_LEFT_TRIGGER.get();
        leftTrigger.addInputListener(this);
        aButton = (DigitalInput) WsInputs.OPERATOR_FACE_DOWN.get();
        aButton.addInputListener(this);
        bButton = (DigitalInput) WsInputs.OPERATOR_FACE_RIGHT.get();
        bButton.addInputListener(this);
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
       
        /**** Motors ****/
        shooterMotor = (WsSpark) WsOutputs.SHOOTERSPEED.get();
        shooterMotor.setCurrentLimit(50,50,0);
        shooterMotor.setCoast(); 
        angleMotor = (WsSpark) WsOutputs.SHOOTERANGLE.get();
        angleMotor.setCurrentLimit(50,50,0);
        feedMotor = (WsSpark) WsOutputs.SHOOTERFEEDMOTOR.get();
        feedMotor.setCurrentLimit(50, 50, 0);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);
        hoodMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        hoodMotor.setBrake();

        /**** Other ****/
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        beamBreakSensor = (DigitalInput) WsInputs.BEAMBREAK_SENSOR.get();
        beamBreakSensor.addInputListener(this);
        curPos = getPosition();
        resetState();
    }

    @Override
    public void inputUpdate(Input source) {
        if (leftBumper.getValue()){
            shooterState = shooterType.SPEAKER;
        } else if (dpadUp.getValue()) {
            shooterState = shooterType.AMP;
        } else if (rightBumper.getValue()){
            shooterState = shooterType.INTAKE;
        } else if (dpadDown.getValue()){
            shooterState = shooterType.OUTTAKE;
        } else {
            shooterState = shooterType.OFF;
        }
    }


    @Override
    public void update() {
        robot_Distance = swerve.getDistanceFromAmp();

        switch (shooterState) {
            case SPEAKER:
                goalVel = getTargetSpeed(robot_Distance);
                shooterEnable = true;
                retract = true;
                if(pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget){
                    feedMotorOutput = 0.5;
                }else{
                    feedMotorOutput = 0.0;
                }
                intakeMotorOutput = 0.0;
                break;
            case AMP:
                goalVel = ShooterConstants.AMP_SPEED;
                if (pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget()) {
                    feedMotorOutput = 0.5;
                } else {
                    feedMotorOutput = 0.0;
                }
                intakeMotorOutput = 0.0;
                break;

            case INTAKE:
                feedMotorOutput = 0.5;
                intakeMotorOutput = 0.5;
                shooterEnable = false;
                break;

            case OUTTAKE:
                feedMotorOutput = -0.5;
                intakeMotorOutput = -0.5;
                shooterEnable = false;
                break;

            case OFF:
            default:
                feedMotorOutput = 0.0;
                intakeMotorOutput = 0.0;
                shooterEnable = false;
                break;
        }

        // Shooter control system
        if (shooterEnable) {
            goalVel = Math.min (goalVel, ShooterConstants.MAX_VEL);
            goalVel = Math.max(goalVel, -ShooterConstants.MAX_VEL);

            curVel = getVelocity();
            velErr = goalVel - curVel;

            shooterOut = goalVel * ShooterConstants.kF + velErr * ShooterConstants.kP;

            shooterMotor.setSpeed(shooterOut);
        } else {
            shooterMotor.setSpeed(0);
        }

        // Pivot control system
        goalPos = Math.min(goalPos, ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0);  // don't command a position higher than the soft stop
        goalPos = Math.max(goalPos, ArmConstants.ZERO_OFFSET * Math.PI / 180.0);  // don't command a position lower than the soft stop
        curPosErr = goalPos - curPos;
        posOut = curPosErr * 0.8;
        if (curPos <= ArmConstants.ZERO_OFFSET * Math.PI / 180.0 ){
            posOut = Math.max(0, posOut);
        } else if (curPos >= ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0){
            posOut = Math.min(0, posOut);
        }

        // Hood control system
        if (!retract && hoodMotor.getPosition() < 1.75){
            hoodOutput = 0.15;
        } else if (hoodMotor.getPosition() > 0.1){
            hoodOutput = -0.15;
        } else {
            hoodOutput = 0.0;
        }

        hoodMotor.setSpeed(hoodOutput);
        intakeMotor.setSpeed(intakeMotorOutput);
        feedMotor.setSpeed(feedMotorOutput);
        angleMotor.setSpeed(posOut);

        // Hood values
        SmartDashboard.putNumber("hood position", hoodMotor.getPosition());
        SmartDashboard.putNumber("hood output", hoodOutput);
        SmartDashboard.putBoolean("hood at target", hoodIsAtTarget());

        // Feed values
        SmartDashboard.putNumber("feed output", feedMotorOutput);
        SmartDashboard.putNumber("intake output", intakeMotorOutput);

        // Shooter values
        SmartDashboard.putNumber("shooter target", goalVel);
        SmartDashboard.putNumber("shooter output", shooterOut);
        SmartDashboard.putNumber("shooter velocity error", curVelErr);
        SmartDashboard.putBoolean("shooter at target", shooterIsAtTarget());
        SmartDashboard.putNumber("shooter velocity", getVelocity());

        // Pivot values
        SmartDashboard.putNumber("goal position", goalPos);
        SmartDashboard.putNumber("shooter position", getPosition());
        SmartDashboard.putNumber("shooter curPosError", curPosErr);
        SmartDashboard.putNumber("Output", posOut);
        SmartDashboard.putBoolean("pivot at target", pivotIsAtTarget());
    }
    
    @Override
    public void resetState() {
        shooterEnable = false;
        goalVel = 0.0;
        hoodOutput = 0.0;
        retract = true;
        curPos = getPosition();
        curVel = shooterMotor.getVelocity();
        goalPos = curPos;
        goalVel = 0;
        posOut = 0;
        feedMotorOutput = 0.0;
        intakeMotorOutput = 0.0;
        shooterState = shooterType.OFF;
    }

    public double getTargetSpeed(double distance){
        for(int i = 0; i < distanceMarks.length; i++){
            if((distance >= distanceMarks[i]) && (distance >= distanceMarks[i+1])){
                indexes[0] = i;
                indexes[1] = i+1;
            }
        }

        return (double)((speeds[indexes[0]]+((speeds[indexes[1]] - speeds[indexes[0]]) * 
                    ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])))));
    }

    public double getTargetAngle(double distance){
        return angles[indexes[0]] + (((angles[indexes[1]] - angles[indexes[0]])) 
            * ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])));
    }

    public void setShooterState(shooterType currentState){
        shooterState = currentState;
    }

    public void setShooterEnable(boolean shooterEnable){
        this.shooterEnable = shooterEnable;
    }
   
   public void setRetract(boolean retract){
        this.retract = retract;
   }
   
   public boolean angleAtTarget(){
        double currentAngle = angleMotor.getPosition();
        robot_Distance = swerve.getDistanceFromSpeaker();
        double angleNeeded = getTargetAngle(robot_Distance);
        if (Math.abs(angleNeeded-currentAngle) < angleDifference){
            return true;
        }
        else{
            return false;
        }
    }

    public double getVelocity() {
        return shooterMotor.getVelocity() * ShooterConstants.RATIO * 2 * Math.PI / 60.0;
    }

    public boolean pivotIsAtTarget() {
        return Math.abs(curPosErr) < ArmConstants.POS_DB && Math.abs(curVel) < ArmConstants.VEL_DB;
    }

    public Boolean hoodIsAtTarget(){
        return (retract && hoodMotor.getPosition() < 0.1) || (!retract && hoodMotor.getPosition() > 1.9);  // TODO: Check position travel limits
    }

    public Boolean shooterIsAtTarget(){
        return Math.abs(goalVel - getVelocity()) < 20;
    }

    public double getPosition(){
        return (angleMotor.getPosition() * 2 * Math.PI / ArmConstants.RATIO) + ArmConstants.ZERO_OFFSET * Math.PI / 180.0;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
    
    @Override
    public void selfTest() {
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
}
