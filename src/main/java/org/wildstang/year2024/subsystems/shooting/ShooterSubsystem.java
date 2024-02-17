package org.wildstang.year2024.subsystems.shooting;

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

    public WsVision limelight;
    public WsSpark shooterMotor;
    public WsSpark angleMotor;
    public double robotDistance;
    public double motorSpeed;
    public DigitalInput shootButton;
    public double motorAngle;
    public AbsoluteEncoder absEncoder;
    public SwerveDrive swerveDrive;
    

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

    @Override
    public void inputUpdate(Input source) {
        if(shootButton.getValue()){
         
            motorSpeed = getSpeed(robotDistance);
            motorAngle = getAngle(robotDistance);
            //robotDistance = limelight.getDistanceFromAprilTag();
            setShooterSpeed(true, robotDistance);
            
        }else if(!shootButton.getValue()){
            setShooterSpeed(false,robotDistance);
        }
    }

    @Override
    public void init() {
        //example
        absEncoder = shooterMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        shootButton = (DigitalInput) WsInputs.OPERATOR_LEFT_TRIGGER.get();
        shootButton.addInputListener(this);
        shooterMotor = (WsSpark) WsOutputs.SHOOTERSPEED.get();
        shooterMotor.setCurrentLimit(50,50,0);
        angleMotor = (WsSpark) WsOutputs.SHOOTERANGLE.get();
        angleMotor.setCurrentLimit(50,50,0);
    }

    @Override
    public void update() {
        
        shooterMotor.setSpeed(motorSpeed);
        angleMotor.setPosition(motorAngle);


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
