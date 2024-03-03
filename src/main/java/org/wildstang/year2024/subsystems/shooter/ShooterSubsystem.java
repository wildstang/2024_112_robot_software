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
    
    private DigitalInput leftBumper, rightBumper;
    private DigitalInput dpadRight, dpadLeft, dpadUp, dpadDown;

    private DigitalInput beamBreakSensor;
    public AbsoluteEncoder pivotEncoder;

    public SwerveDrive swerve;

    private double feedMotorOutput, intakeMotorOutput;

    private boolean retract;
    private double hoodPos;
    private double hoodOutput;

    private boolean shooterEnable;
    private double distance;
    private double goalVel, curVel, velErr;
    private double shooterOut;

    private double goalPos, curPos, posErr;
    private double posOut;

    public double[] speeds = {0.5, 0.6, 0.7, 0.8, 0.9, 1};

    public double[] angles = {1,1,1,1,1,1};

    public double[] distanceMarks = {1,1,1,1,1,1};

    public int[] indexes = new int[2];

    @Override
    public void init() {
        /**** Abs Encoders ****/
        pivotEncoder = angleMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        
        /**** Button Inputs ****/
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
        shooterMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTERSPEED);
        angleMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTERANGLE);
        feedMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.SHOOTERFEEDMOTOR);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);
        hoodMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);

        /**** Other ****/
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        beamBreakSensor = (DigitalInput) Core.getInputManager().getInput(WsInputs.BEAMBREAK_SENSOR);
        beamBreakSensor.addInputListener(this);

        resetState();
    }

    @Override
    public void inputUpdate(Input source) {
        if (leftBumper.getValue()){
            shooterState = shooterType.INTAKE;
        } else if (dpadUp.getValue()) {
            shooterState = shooterType.AMP;
        } else if (rightBumper.getValue()){
            shooterState = shooterType.SPEAKER;
        } else if (dpadDown.getValue()){
            shooterState = shooterType.OUTTAKE;
        } else {
            shooterState = shooterType.OFF;
        }
    }


    @Override
    public void update() {
        curVel = getShooterVelocity();
        curPos = getPivotPosition();
        hoodPos = hoodMotor.getPosition();

        switch (shooterState) {
            case SPEAKER:
                distance = swerve.getDistanceFromSpeaker();
                goalVel = getTargetSpeed(distance);
                goalPos = getTargetAngle(distance);
                shooterEnable = true;
                retract = true;
                if(pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget()){
                    feedMotorOutput = FeedConstants.FEED_SPEED;
                }else{
                    feedMotorOutput = 0.0;
                }
                intakeMotorOutput = 0.0;
                break;
            case AMP:
                goalVel = ShooterConstants.AMP_SPEED;
                goalPos = ArmConstants.AMP_POS;
                if (pivotIsAtTarget() && shooterIsAtTarget() && hoodIsAtTarget() && swerve.isAtTarget()) {
                    feedMotorOutput = FeedConstants.FEED_SPEED;
                } else {
                    feedMotorOutput = 0.0;
                }
                intakeMotorOutput = 0.0;
                break;

            case INTAKE:
                goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                feedMotorOutput = FeedConstants.FEED_IN_SPEED;
                intakeMotorOutput = FeedConstants.INTAKE_IN_SPEED;
                shooterEnable = false;
                break;

            case OUTTAKE:
                goalPos = Math.max(curPos, ArmConstants.MIN_INTAKE_POS);
                goalPos = Math.min(curPos, ArmConstants.MAX_INTAKE_POS);
                goalVel = ShooterConstants.OUTTAKE_SPEED;
                feedMotorOutput = FeedConstants.FEED_OUT_SPEED;
                intakeMotorOutput = FeedConstants.INTAKE_OUT_SPEED;
                shooterEnable = true;
                break;

            case OFF:
                feedMotorOutput = 0.0;
                intakeMotorOutput = 0.0;
                shooterEnable = false;
                break;
        }

        // Shooter control system
        goalVel = Math.min(goalVel, ShooterConstants.MAX_VEL);
        goalVel = Math.max(goalVel, -ShooterConstants.MAX_VEL);

        velErr = goalVel - curVel;

        shooterOut = goalVel * ShooterConstants.kF + velErr * ShooterConstants.kP;
        if (shooterEnable) {
            shooterMotor.setSpeed(shooterOut);
        } else {
            shooterMotor.setSpeed(0);
        }

        // Pivot control system
        goalPos = Math.min(goalPos, ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0);  // don't command a position higher than the soft stop
        goalPos = Math.max(goalPos, ArmConstants.ZERO_OFFSET * Math.PI / 180.0);  // don't command a position lower than the soft stop
        posErr = goalPos - curPos;
        posOut = posErr * ArmConstants.kP;
        if (curPos <= ArmConstants.ZERO_OFFSET * Math.PI / 180.0 ){
            posOut = Math.max(0, posOut);
        } else if (curPos >= ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0){
            posOut = Math.min(0, posOut);
        }

        // Hood control system
        if (!retract){  //  && hoodPos < HoodConstants.EXTEND_POS  TODO: decide if we want to stall or brake the motor
            hoodOutput = HoodConstants.EXTEND_SPEED;
        } else if (hoodPos > HoodConstants.RETRACT_POS){
            hoodOutput = HoodConstants.RETRACT_SPEED;
        } else {
            hoodOutput = 0.0;
        }

        hoodMotor.setSpeed(hoodOutput);
        intakeMotor.setSpeed(intakeMotorOutput);
        feedMotor.setSpeed(feedMotorOutput);
        angleMotor.setSpeed(posOut);

        // Hood values
        SmartDashboard.putNumber("hood position", hoodPos);
        SmartDashboard.putNumber("hood output", hoodOutput);
        SmartDashboard.putBoolean("hood at target", hoodIsAtTarget());

        // Feed values
        SmartDashboard.putNumber("feed output", feedMotorOutput);
        SmartDashboard.putNumber("intake output", intakeMotorOutput);

        // Shooter values
        SmartDashboard.putNumber("shooter target", goalVel);
        SmartDashboard.putNumber("shooter output", shooterOut);
        SmartDashboard.putNumber("shooter velocity error", velErr);
        SmartDashboard.putBoolean("shooter at target", shooterIsAtTarget());
        SmartDashboard.putNumber("shooter velocity", curVel);

        // Pivot values
        SmartDashboard.putNumber("goal position", goalPos);
        SmartDashboard.putNumber("shooter position", curPos);
        SmartDashboard.putNumber("shooter curPosError", posErr);
        SmartDashboard.putNumber("Output", posOut);
        SmartDashboard.putBoolean("pivot at target", pivotIsAtTarget());
    }
    
    @Override
    public void resetState() {
        shooterEnable = false;
        goalVel = 0.0;
        hoodOutput = 0.0;
        retract = true;
        curPos = getPivotPosition();
        curVel = getShooterVelocity();
        goalPos = curPos;
        goalVel = 0;
        posOut = 0;
        feedMotorOutput = 0.0;
        intakeMotorOutput = 0.0;
        shooterState = shooterType.OFF;
    }

    public double getPivotPosition(){
        return ((pivotEncoder.getPosition() + ArmConstants.ZERO_OFFSET) % 360) * Math.PI / 180.0;
        // return (angleMotor.getPosition() * 2 * Math.PI / ArmConstants.RATIO) + ArmConstants.ZERO_OFFSET * Math.PI / 180.0;
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

    public void setShooterState(shooterType newState){
        shooterState = newState;
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity() * ShooterConstants.RATIO * 2 * Math.PI / 60.0;
    }

    public boolean pivotIsAtTarget() {
        return Math.abs(posErr) < ArmConstants.POS_DB && Math.abs(curVel) < ArmConstants.VEL_DB;
    }

    public Boolean hoodIsAtTarget(){
        return (retract && hoodPos < HoodConstants.RETRACT_POS) || (!retract && hoodPos > HoodConstants.EXTEND_POS);
    }

    public Boolean shooterIsAtTarget(){
        return Math.abs(goalVel - getShooterVelocity()) < ShooterConstants.VEL_DB;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
    
    @Override
    public void selfTest() {
    }

    public double getVelocityTarget(double curVel, double posErr){
        if (posErr > 0){
            if (posErr <= curVel * Math.abs(curVel / ArmConstants.MAX_ACC) * .5){
                return curVel - (ArmConstants.MAX_ACC * ArmConstants.DELTA_T);
            } else if (curVel < ArmConstants.MAX_VEL){
                return Math.min(curVel + ArmConstants.MAX_ACC * ArmConstants.DELTA_T, ArmConstants.MAX_VEL);
            } else{
                return ArmConstants.MAX_VEL;
            }
        } else {
            if (posErr >= curVel * Math.abs(curVel / ArmConstants.MAX_ACC) * .5){
                return curVel + (ArmConstants.MAX_ACC * ArmConstants.DELTA_T);
            } else if (curVel > -ArmConstants.MAX_VEL){
                return Math.max(curVel - ArmConstants.MAX_ACC * ArmConstants.DELTA_T, -ArmConstants.MAX_VEL);
            } else{
                return -ArmConstants.MAX_VEL;
            }
        }
    }
}
