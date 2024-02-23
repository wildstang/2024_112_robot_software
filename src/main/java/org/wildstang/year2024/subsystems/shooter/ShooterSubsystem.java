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

import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

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

    
    public WsSpark angleMotor;
    public WsSpark shooterMotor;
    public WsSpark feedMotor;
    public double robot_Distance;
    public double motorSpeed;
    public DigitalInput leftTrigger;
    public double motorAngle;
    public AbsoluteEncoder absEncoderShooter;
    public AbsoluteEncoder absEncoderFeed;
    public SwerveDrive drive;
    private DigitalInput beamBreakSensor;
    private WsSpark intakeMotor;
    private double feedMotorSpeed = 0;
    private double intakeMotorSpeed;
    private DigitalInput aButton;
    private DigitalInput bButton;
    
    

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
            motorSpeed = getSpeed(robotDistance);
            motorAngle = getAngle(robotDistance);
        }
        else{
           motorSpeed = 0;
           motorAngle = 0;
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
            setShooterSpeed(true, robot_Distance);
           
        }
        else if(!leftTrigger.getValue()){
            setShooterSpeed(false, robot_Distance);
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


    }

    @Override
    public void init() {
        /**** Abs Encoders ****/
        absEncoderShooter = shooterMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        absEncoderFeed = feedMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        
        /**** Button Inputs ****/
        leftTrigger = (DigitalInput) WsInputs.DRIVER_LEFT_TRIGGER.get();
        leftTrigger.addInputListener(this);
        aButton = (DigitalInput) WsInputs.OPERATOR_FACE_DOWN.get();
        aButton.addInputListener(this);
        bButton = (DigitalInput) WsInputs.OPERATOR_FACE_RIGHT.get();
        bButton.addInputListener(this);
        

        /**** Motors ****/
        shooterMotor = (WsSpark) WsOutputs.SHOOTERSPEED.get();
        shooterMotor.setCurrentLimit(50,50,0);
        angleMotor = (WsSpark) WsOutputs.SHOOTERANGLE.get();
        angleMotor.setCurrentLimit(50,50,0);
        feedMotor = (WsSpark) WsOutputs.SHOOTERFEEDMOTOR.get();
        feedMotor.setCurrentLimit(50, 50, 0);
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);

        /**** Other ****/
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        beamBreakSensor = (DigitalInput) WsInputs.BEAMBREAK_SENSOR.get();
        beamBreakSensor.addInputListener(this);
    }

    @Override
    public void update() {
        intakeMotor.setSpeed(intakeMotorSpeed);
        shooterMotor.setSpeed(motorSpeed);
       angleMotor.setPosition(motorAngle);
        feedMotor.setSpeed(feedMotorSpeed);
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
