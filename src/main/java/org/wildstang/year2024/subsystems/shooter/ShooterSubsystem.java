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
    /* 1. rename this ^ to be whatever your file is called
     * 2. go to getName() and change the string "template" to a string that describes this subsystem
     * 3. add any variables you need under this comment block (inputs, outputs, doubles/booleans/ints)
     * 4. Initialize any inputs and outputs under init()
     * 5. add your code where it should be in inputUpdate() and update()
     */

     //example
    //  private DigitalInput button;
    //  private WsSpark motor;

    private enum feedType {SPEAKER, AMP, INTAKE, OUTTAKE, OFF};
    public WsSpark angleMotor;
    public WsSpark shooterMotor;
    public WsSpark feedMotor;
    public double robot_Distance;
    public double motorSpeed;
    public DigitalInput leftTrigger;
    public double motorAngle;
    private DigitalInput dpadRight;
    private DigitalInput dpadLeft;
    public AbsoluteEncoder absEncoderShooter;
    public AbsoluteEncoder absEncoderFeed;
    public AbsoluteEncoder absEncoderAngle;
    public SwerveDrive drive;
    private double velocityDifference;
    private double angleDifference;
    private DigitalInput beamBreakSensor;
    private WsSpark intakeMotor;
    private double feedMotorSpeed = 0;
    private double intakeMotorSpeed;
    private DigitalInput aButton;
    private DigitalInput bButton;
    private boolean retract;
    private double goalVel, curVel, velErr;
    private double out;
    private Boolean shooterEnable;
    private DigitalInput leftBumper, rightBumper, dpadUp;
    double goalPos, curPos, curPosErr;
    double curVelErr;
    double curOut = 0;
    private WsSpark ampHoodMotor;
    private double ampHoodSpeed;

    public double[] speeds = {0.5, 0.6, 0.7, 0.8, 0.9, 1};

    public double[] angles = {1,1,1,1,1,1};

    public double[] distanceMarks = {1,1,1,1,1,1};

    public int[] indexes = new int[2];

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

   public void setShooterSpeed(boolean enable){
        shooterEnable = enable;
   }
   public void setAngle(boolean angleAllowed, double ampAngle){
    if (angleAllowed){
        motorAngle = ampAngle;
    }
    else{
        motorAngle = 0;
    }
   }
   
   public boolean velocityAtTarget(){
    double currentVelocity = shooterMotor.getVelocity();
    robot_Distance = drive.getDistanceFromSpeaker();
    double velocityNeeded1 = getTargetSpeed(robot_Distance);
    if (Math.abs(velocityNeeded1 - currentVelocity) < velocityDifference){
        return true;
    }
    else{
        return false;
    }
   }
   
   public void setRetract(boolean retract){
        this.retract = retract;
   }
   
   public boolean angleAtTarget(){
    double currentAngle = angleMotor.getPosition();
    robot_Distance = drive.getDistanceFromSpeaker();
    double angleNeeded = getTargetAngle(robot_Distance);
    if (Math.abs(angleNeeded-currentAngle) < angleDifference){
        return true;
    }
    else{
        return false;
    }
   }
   public void setNotepathSpeed(boolean speedForward, boolean speedBackwards){
    if (speedForward == true && speedBackwards == false){
        feedMotorSpeed = 1;
        intakeMotorSpeed = 1;
    }
    else if (speedForward == false && speedBackwards == false){
        feedMotorSpeed = -1;
        intakeMotorSpeed = -1;
    }
    else if (speedForward == false && speedBackwards == true){
        feedMotorSpeed = 1;
    }
    else{
        feedMotorSpeed = 0;
        intakeMotorSpeed = 0;
    }
}

    @Override
    public void inputUpdate(Input source) {
        robot_Distance = drive.getDistanceFromSpeaker();
        if(leftTrigger.getValue()){
            setShooterSpeed(true);
           
        }
        else if(!leftTrigger.getValue()){
            setShooterSpeed(false);
        }
        if (aButton.getValue() && beamBreakSensor.getValue() == false){
            setNotepathSpeed(true, true);
        }
        //bButton reverses speed and sets both motor speeds to -1
        else if (bButton.getValue()){
            setNotepathSpeed(false,false);
        }
        //sets both motor speeds to 0
        else{
            setNotepathSpeed(true, false);
        }
        if (source == dpadUp) {
            if (dpadUp.getValue()){
            goalVel = 150;
            shooterEnable = true;
            retract = false;
            goalPos = 100 * Math.PI / 180.0;  // TODO: verify amp scoring angle
            }
            else {
                goalPos = 35 * Math.PI / 180.0;  // TODO: verify podium angle
                retract = true;
            }
        }
        else if (source == leftBumper && leftBumper.getValue()){
        goalVel = 445;  // 445
        goalPos = 35 * Math.PI / 180.0;
        shooterEnable = true;
       } else if (source == rightBumper && rightBumper.getValue()) {
        goalVel = -300;
        shooterEnable = true;
       } else {
        shooterEnable = false;
        }


    }

    @Override
    public void init() {
        /**** Abs Encoders ****/
        absEncoderShooter = shooterMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoderFeed = feedMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
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
       

        /**** Motors ****/
        shooterMotor = (WsSpark) WsOutputs.SHOOTERSPEED.get();
        shooterMotor.setCurrentLimit(50,50,0);
        shooterMotor.setCoast(); 
        angleMotor = (WsSpark) WsOutputs.SHOOTERANGLE.get();
        angleMotor.setCurrentLimit(50,50,0);
        feedMotor = (WsSpark) WsOutputs.SHOOTERFEEDMOTOR.get();
        feedMotor.setCurrentLimit(50, 50, 0);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);
        ampHoodMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.AMPHOOD);
        ampHoodMotor.setBrake();

        /**** Other ****/
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        beamBreakSensor = (DigitalInput) WsInputs.BEAMBREAK_SENSOR.get();
        beamBreakSensor.addInputListener(this);
        curPos = getPosition();
        resetState();
    }

    @Override
    public void update() {
        intakeMotor.setSpeed(intakeMotorSpeed);
        angleMotor.setPosition(motorAngle);
        feedMotor.setSpeed(feedMotorSpeed);
        SmartDashboard.putNumber("shooter target", goalVel);
        SmartDashboard.putNumber("shooter speed", out);
        SmartDashboard.putBoolean("shooter at target", ampIsAtTarget());
        SmartDashboard.putNumber("shooter velocity", getVelocity());
        switch (feedState) {
            case SPEAKER:
            robot_Distance = drive.getDistanceFromSpeaker();
            goalVel = getTargetSpeed(robot_Distance);
            shooterEnable = true;
            case AMP:
            robot_Distance = drive.getDistanceFromAmp();
            goalVel = getTargetSpeed(robot_Distance);
            if (ampIsAtTarget() && shooterisAtTarget() && ampHoodisAtTarget()) {
                feedMotorSpeed = 0.5;
            } else {
                feedMotorSpeed = 0.0;
            }
            intakeMotorSpeed = 0.0;
            break;

            case INTAKE:
                feedMotorSpeed = -0.5;
                intakeMotorSpeed = 0.0;
                break;

            case OUTTAKE:
                feedMotorSpeed = -0.5;
                intakeMotorSpeed = -0.5;
                break;

            case OFF:
            default:
                feedMotorSpeed = 0.0;
                intakeMotorSpeed = 0.0;
                break;
        }
        goalPos = Math.min(goalPos, ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0);  // don't command a position higher than the soft stop
        goalPos = Math.max(goalPos, ArmConstants.ZERO_OFFSET * Math.PI / 180.0);  // don't command a position lower than the soft stop
        curPosErr = goalPos - curPos;
        curOut = curPosErr * 0.8;
        if (curPos <= ArmConstants.ZERO_OFFSET * Math.PI / 180.0 ){
            curOut = Math.max(0, curOut);
        } else if (curPos >= ArmConstants.SOFT_STOP_HIGH * Math.PI / 180.0){
            curOut = Math.min(0, curOut);
            SmartDashboard.putNumber("shooter curPosError", curPosErr);
            SmartDashboard.putNumber("shooter goal velocity", goalVel);
            SmartDashboard.putNumber("shooter velocity error", curVelErr);
            SmartDashboard.putNumber("Output", curOut);
            SmartDashboard.putBoolean("pivot at target", ampIsAtTarget());
        }
        if (!retract && ampHoodMotor.getPosition() < 1.75){
            ampHoodSpeed = 0.15;
        } else if (ampHoodMotor.getPosition() > 0.1){
            ampHoodSpeed = -0.15;
        } else {
            ampHoodSpeed = 0.0;
        }
       ampHoodMotor.setSpeed(ampHoodSpeed);
       SmartDashboard.putNumber("hood position", ampHoodMotor.getPosition());
       SmartDashboard.putNumber("amp speed", ampHoodSpeed);
       SmartDashboard.putBoolean("hood at target", ampHoodisAtTarget());

    }
    

    public double getVelocity() {
        return shooterMotor.getVelocity() * ShooterConstants.RATIO * 2 * Math.PI / 60.0;
    }
    public boolean ampIsAtTarget() {
        return Math.abs(curPosErr) < ArmConstants.POS_DB && Math.abs(curVel) < ArmConstants.VEL_DB;
    }
    public Boolean ampHoodisAtTarget(){
        return (retract && ampHoodMotor.getPosition() < 0.1) || (!retract && ampHoodMotor.getPosition() > 1.9);  // TODO: Check position travel limits
    }

    public Boolean shooterisAtTarget(){
        return Math.abs(goalVel - getVelocity()) < 20;
    }
    public double getPosition(){
        return (angleMotor.getPosition() * 2 * Math.PI / ArmConstants.RATIO) + ArmConstants.ZERO_OFFSET * Math.PI / 180.0;
    }


    
    @Override
    public String getName() {
        //
        return "Shooter";
    }
    @Override
    public void resetState() {
        shooterEnable = false;
        goalVel = 0.0;
        ampHoodSpeed = 0.0;
        retract = true;
    }
    @Override
    public void selfTest() {
    }
}
