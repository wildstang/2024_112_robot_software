package org.wildstang.year2024.subsystems.shooter;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import org.wildstang.year2024.subsystems.swerve.SwerveDrive;
import org.wildstang.year2024.subsystems.targeting.WsVision;

public class ShooterSubsystem implements Subsystem{
    /* 1. rename this ^ to be whatever your file is called
     * 2. go to getName() and change the string "template" to a string that describes this subsystem
     * 3. add any variables you need under this comment block (inputs, outputs, doubles/booleans/ints)
     * 4. Initialize any inputs and outputs under init()
     * 5. add your code where it should be in inputUpdate() and update()
     */

     //example
    //  private DigitalInput button;
    //  private WsSpark motor;

    
    public WsSpark leftAngleMotor;
    public WsSpark rightAngleMotor;
    public WsSpark leftShooterMotor;
    public WsSpark rightShooterMotor;
    public WsSpark feedMotor;
    public double robot_Distance;
    public double leftMotorSpeed;
    public double rightMotorSpeed;
    public DigitalInput rightBumperShootButton;
    public DigitalInput leftBumperFeedButton;
    public double leftMotorAngle;
    public double rightMotorAngle;
    public AbsoluteEncoder absEncoderShooter1;
    public AbsoluteEncoder absEncoderShooter2;
    public AbsoluteEncoder absEncoderFeed;
    public SwerveDrive drive;
    

    public double[] speeds = {0.5, 0.6, 0.7, 0.8, 0.9, 1};

    public double[] angles = {1,1,1,1,1,1};

    public double[] distanceMarks = {1,1,1,1,1,1};

    public int[] indexes = new int[2];

    public double getSpeed(double distance){
        for(int i = 0; i < distanceMarks.length; i++){
            if((distance >= distanceMarks[i]) && (distance >= distanceMarks[i+1])){
                indexes[0] = i;
                indexes[1] = i+1;
                
                
            }
        }

        return (double)((speeds[indexes[0]]+((speeds[indexes[1]] - speeds[indexes[0]]) * 
                    ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])))));


    }

    public double getAngle(double distance){
        return angles[indexes[0]] + (((angles[indexes[1]] - angles[indexes[0]])) 
            * ((distance - distanceMarks[indexes[0]]) / (distanceMarks[indexes[1]] - distanceMarks[indexes[0]])));
    }

   public void setShooterSpeed(boolean shootAllowed, double robotDistance){
        if (shootAllowed){
            leftMotorSpeed = getSpeed(robotDistance);
            rightMotorSpeed = -getSpeed(robotDistance);
            leftMotorAngle = getAngle(robotDistance);
            rightMotorAngle = -getAngle(robotDistance);
        }
        else{
           leftMotorSpeed = 0;
           rightMotorSpeed = 0;
           leftMotorAngle = 0; 
           rightMotorAngle = 0;
        }
   }

    @Override
    public void inputUpdate(Input source) {
        if(leftBumperFeedButton.getValue()){
            feedMotor.setSpeed(0.5);
            
        }else if(!leftBumperFeedButton.getValue()){
            feedMotor.setSpeed(0);
        }
    }

    @Override
    public void init() {
        /**** Abs Encoders ****/
        absEncoderShooter1 = leftShooterMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoderShooter2 = rightShooterMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoderFeed = feedMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        
        /**** Button Inputs ****/
        leftBumperFeedButton = (DigitalInput) WsInputs.OPERATOR_LEFT_SHOULDER.get();
        leftBumperFeedButton.addInputListener(this);
        rightBumperShootButton = (DigitalInput) WsInputs.OPERATOR_RIGHT_SHOULDER.get();
        rightBumperShootButton.addInputListener(this);

        /**** Motors ****/
        leftShooterMotor = (WsSpark) WsOutputs.LEFTSHOOTERSPEED.get();
        leftShooterMotor.setCurrentLimit(50,50,0);
        rightShooterMotor = (WsSpark) WsOutputs.RIGHTSHOOTERSPEED.get();
        rightShooterMotor.setCurrentLimit(50,50,0);
        leftAngleMotor = (WsSpark) WsOutputs.LEFTSHOOTERANGLE.get();
        leftAngleMotor.setCurrentLimit(50,50,0);
        rightAngleMotor = (WsSpark) WsOutputs.RIGHTSHOOTERANGLE.get();
        rightAngleMotor.setCurrentLimit(50,50,0);
        feedMotor = (WsSpark) WsOutputs.SHOOTERFEEDMOTOR.get();
        feedMotor.setCurrentLimit(50, 50, 0);

        /**** Other ****/
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {
        robot_Distance = drive.getDistanceFromSpeaker();
        while(rightBumperShootButton.getValue()){
            setShooterSpeed(true, robot_Distance);
            rightShooterMotor.setSpeed(rightMotorSpeed);
            leftShooterMotor.setSpeed(-leftMotorSpeed);
            leftAngleMotor.setPosition(-leftMotorAngle);
            rightAngleMotor.setPosition(rightMotorAngle);
        }
        if(!rightBumperShootButton.getValue()){
            setShooterSpeed(false, robot_Distance);
            rightShooterMotor.setSpeed(rightMotorSpeed);
            leftShooterMotor.setSpeed(leftMotorSpeed);
            leftAngleMotor.setPosition(leftMotorAngle);
            rightAngleMotor.setPosition(rightMotorAngle);
        }
       


    }

    
    @Override
    public String getName() {
        //
        return "template";
    }
    @Override
    public void resetState() {
    }
    @Override
    public void selfTest() {
    }
}
