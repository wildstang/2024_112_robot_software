package org.wildstang.year2024.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import java.util.Optional;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.targeting.LimeConsts;
import org.wildstang.year2024.subsystems.targeting.WsVision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: SwerveDrive
 * inputs: driver left joystick x/y, right joystick x, right trigger, right bumper, select, face buttons all, gyro
 * outputs: four swerveModule objects
 * description: controls a swerve drive for four swerveModules through autonomous and teleoperated control
 */
public class SwerveDrive extends SwerveDriveTemplate {

    private AnalogInput leftStickX;//translation joystick x
    private AnalogInput leftStickY;//translation joystick y
    private AnalogInput rightStickX;//rot joystick
    private AnalogInput rightTrigger;//thrust
    private AnalogInput leftTrigger;//scoring autodrive
    private DigitalInput rightBumper;//
    private DigitalInput leftBumper;//hp station pickup
    private DigitalInput select;//gyro reset
    private DigitalInput start;//
    private DigitalInput faceUp;//rotation lock 0 degrees
    private DigitalInput faceRight;//rotation lock 90 degrees
    private DigitalInput faceLeft;//rotation lock 270 degrees
    private DigitalInput faceDown;//rotation lock 180 degrees
    private DigitalInput dpadLeft;//defense mode
    private DigitalInput rightStickButton;//auto drive override

    private double xSpeed;
    private double ySpeed;
    private double rotSpeed;
    private double thrustValue;
    private double derateValue;
    private boolean rotLocked;
    private boolean isFieldCentric;
    private double rotTarget;
    private double pathVel;
    private double pathHeading;
    private double pathAccel;
    private double pathTarget;
    private double pathXOffset = 0;
    private double pathYOffset = 0;

    private final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    private SwerveDrivePoseEstimator poseEstimator;
    private Timer autoTimer = new Timer();
    public Optional<Alliance> station;

    private WsVision limelight;
    private LimeConsts lc;

    public enum driveType {TELEOP, AUTO, CROSS, SPEAKER, AMP, STAGE};
    public driveType driveState;


    public String getClosestChain(){
        double robotX = getPosX();
        double robotY = getPosY();
        if(station.orElse(null).equals(Alliance.Red)){
            double distanceFromTag13 = distanceFrom(robotX, DriveConstants.tag13[0], robotY, DriveConstants.tag13[1]);
            double distanceFromTag12 = distanceFrom(robotX, DriveConstants.tag12[0], robotY, DriveConstants.tag12[1]);
            double distanceFromTag11 = distanceFrom(robotX, DriveConstants.tag11[0], robotY, DriveConstants.tag11[1]);
            double smallest = Math.min(Math.min(distanceFromTag13, distanceFromTag12), distanceFromTag11);

            if(smallest == distanceFromTag11){
                return "tag11";
            }else if(smallest == distanceFromTag12){
                return "tag12";
            }else if(smallest == distanceFromTag13){
                return "tag13";
            }
            
        }else if(station.orElse(null).equals(Alliance.Blue)){
            double distanceFromTag16 = distanceFrom(robotX,DriveConstants.tag16[0],robotY, DriveConstants.tag16[1]);
            double distanceFromTag15 = distanceFrom(robotX,DriveConstants.tag15[0],robotY, DriveConstants.tag15[1]);
            double distanceFromTag14 = distanceFrom(robotX,DriveConstants.tag14[0],robotY, DriveConstants.tag14[1]);
            double smallest = Math.min(Math.min(distanceFromTag16, distanceFromTag15), distanceFromTag14);

            if(smallest == distanceFromTag16){
                return "tag16";
            }else if(smallest == distanceFromTag15){
                return "tag15";
            }else if(smallest == distanceFromTag14){
                return "tag14";
            }
        }
            
        return "none";
    }

    public static double distanceFrom(double x1, double x2, double y1, double y2){
        return (double)(Math.sqrt(Math.pow(x2-x1,2) + Math.pow(y2-y1,2)));
    }

     // Get x Pos and y Pos and calulate angle of turn needed to line up with speaker
     public double getAngleToSpeaker(){
        double xPosition;
        double yPosition;
        double angleToSpeaker = 0;

        if(station.orElse(null).equals(Alliance.Blue)){
            xPosition = lc.BLUE_SPEAKER_X - poseEstimator.getEstimatedPosition().getX();
            yPosition = -poseEstimator.getEstimatedPosition().getY();

            angleToSpeaker = Math.atan(xPosition/yPosition);
            
            
        }else if(station.orElse(null).equals(Alliance.Red)){
            xPosition = poseEstimator.getEstimatedPosition().getX() - lc.RED_SPEAKER_X;
            yPosition = -poseEstimator.getEstimatedPosition().getY();
            
            angleToSpeaker = Math.atan(xPosition/yPosition);
        }
        return angleToSpeaker;
    }

    // Get the distance the robot is from AMP within a 49 inch radius and return the angle and direction the robot needs to drive
    public boolean isInAmpRadius(){

        
        double robotDistance; // Distance of robot from AprilTag

        // Red Alliance April Tag
        if(station.orElse(null).equals(Alliance.Blue)){
            robotDistance = Math.sqrt((Math.pow((Math.pow((lc.AMP_X - poseEstimator.getEstimatedPosition().getX()),2)) + (-poseEstimator.getEstimatedPosition().getY()),2)));
        }else{
            robotDistance = Math.sqrt((Math.pow(((lc.AMP_X+(lc.FIELD_WIDTH-(lc.AMP_X*2))) - poseEstimator.getEstimatedPosition().getX()),2))) + (Math.pow((-poseEstimator.getEstimatedPosition().getY()),2));
        }
        
        if(robotDistance <= lc.RADIUS_OF_AMP_TARGETING_ZONE){
            return true;
        }else{
            return false;
        }
        

    }

    /*public double getDistanceToCenterOfChainPlusOffset(){
        double robotDriveDistance;
            double xPos = poseEstimator.getEstimatedPosition().getX();
            double yPos = poseEstimator.getEstimatedPosition().getY();
            double aprilTagX = poseEstimator.getEstimatedPosition().getX();
            double aprilTagY = poseEstimator.getEstimatedPosition().getY();
            double robotDistance = Math.sqrt(Math.pow(xPos - aprilTagX,2) + Math.pow(yPos - aprilTagY,2));
            double angleAtAprilTag = 0;
            robotDriveDistance = 
                Math.sqrt((
                    (Math.pow(
                        (lc.CORE_OF_STAGE_TO_CHAIN + lc.CLIMBER_OFFSET),2)
                    ) + 
                    (Math.pow(robotDistance,2)) - (2*((lc.CORE_OF_STAGE_TO_CHAIN + lc.CLIMBER_OFFSET)))) * Math.cos(Math.toRadians(angleAtAprilTag)))
        
        return robotDriveDistance;
        

    }*/

    public double[] FindThirdVertex(double sideA, double sideB, double sideC, double[] vertex1, double[] vertex2){
        double angleA = Math.toDegrees(Math.acos(((sideB*sideB)+(sideC*sideC) - (sideA*sideA)) / (2*sideB*sideC))); // Degrees
        
        double directionX = Math.cos(Math.toRadians(angleA));
        double directionY = Math.sin(Math.toRadians(angleA));

        double thirdVertexX = vertex2[0] + sideA * directionX;
        double thirdVertexY = vertex2[1] + sideA * directionY;

        double[] thirdVertex = {thirdVertexX, thirdVertexY};

        return thirdVertex;
    
    }

    public double getPosX(){
        return (double)(poseEstimator.getEstimatedPosition().getX());
    }

    public double getPosY(){
         return (double)(poseEstimator.getEstimatedPosition().getY());
    }

    public double getDistanceFromPose(Pose2d goalPose){
        return Math.sqrt(Math.pow(goalPose.getX() - getPosX(),2) + Math.pow((goalPose.getY() - getPosY()),2));
    }

    @Override
    public void inputUpdate(Input source) {

        //determine if we are in cross or teleop
        if (driveState != driveType.AUTO && dpadLeft.getValue()) {
            driveState = driveType.CROSS;
            for (int i = 0; i < modules.length; i++) {
                modules[i].setDriveBrake(true);
            }
            this.swerveSignal = new SwerveSignal(new double[]{0, 0, 0, 0 }, swerveHelper.setCross().getAngles());
        }
        else if (driveState == driveType.CROSS || driveState == driveType.AUTO) {
            driveState = driveType.TELEOP;
        }

        //get x and y speeds
        xSpeed = swerveHelper.scaleDeadband(leftStickX.getValue(), DriveConstants.DEADBAND);
        ySpeed = swerveHelper.scaleDeadband(leftStickY.getValue(), DriveConstants.DEADBAND);
        
        //reset gyro
        if (source == select && select.getValue()) {
            gyro.setYaw(0.0);
            if (rotLocked) rotTarget = 0.0;
        }

        if (source == faceUp || source == faceRight || source == faceDown || source == faceLeft){
            if (faceUp.getValue()){
                if (faceLeft.getValue()){
                    rotTarget = 315.0;
                } else if (faceRight.getValue()){ 
                    rotTarget = 45.0;
                } else  rotTarget = 0.0;
                rotLocked = true;
            } else if (faceDown.getValue()){
                if (faceLeft.getValue()){
                    rotTarget = 225.0;
                } else if (faceRight.getValue()){ 
                    rotTarget = 135.0;
                } else  rotTarget = 180.0;
                rotLocked = true;
            } else if (faceLeft.getValue()){
                rotTarget = 270.0;
                rotLocked = true;
            } else if (faceRight.getValue()){
                rotTarget = 90.0;
                rotLocked = true;
            } else{
                rotLocked = false;
            }
        }

        //get rotational joystick
        rotSpeed = rightStickX.getValue()*Math.abs(rightStickX.getValue());
        rotSpeed = swerveHelper.scaleDeadband(rotSpeed, DriveConstants.DEADBAND);
        rotSpeed *= DriveConstants.ROTATION_SPEED;
        //if the rotational joystick is being used, the robot should not be auto tracking heading
        if (rotSpeed != 0) {
            rotLocked = false;
        }
        
        //assign thrust
        thrustValue = 1 - DriveConstants.DRIVE_THRUST + DriveConstants.DRIVE_THRUST * Math.abs(rightTrigger.getValue());
        derateValue = (DriveConstants.DRIVE_DERATE * Math.abs(leftTrigger.getValue()) + 1);
        xSpeed *= thrustValue/derateValue;
        ySpeed *= thrustValue/derateValue;
        rotSpeed *= thrustValue;

    }
 
    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);
    }

    public void initInputs() {
        limelight = (WsVision) Core.getSubsystemManager().getSubsystem("Ws Vision");
        lc = new LimeConsts();

        leftStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_X);
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        rightStickX.addInputListener(this);
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        select = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_SELECT);
        select.addInputListener(this);
        start = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_START);
        start.addInputListener(this);
        faceUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_UP);
        faceUp.addInputListener(this);
        faceLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_LEFT);
        faceLeft.addInputListener(this);
        faceRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_RIGHT);
        faceRight.addInputListener(this);
        faceDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_DOWN);
        faceDown.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        rightStickButton = (DigitalInput) WsInputs.DRIVER_RIGHT_JOYSTICK_BUTTON.get();
        rightStickButton.addInputListener(this);
    }

    public void initOutputs() {
        //create four swerve modules
        modules = new SwerveModule[]{
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE1), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE1), DriveConstants.FRONT_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE2), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE2), DriveConstants.FRONT_RIGHT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE3), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE3), DriveConstants.REAR_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE4), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE4), DriveConstants.REAR_RIGHT_OFFSET)
        };
        //create default swerveSignal
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
        limelight = (WsVision) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_VISION);
        poseEstimator = new SwerveDrivePoseEstimator(new SwerveDriveKinematics(new Translation2d(DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2),
            new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2)), odoAngle(), odoPosition(), new Pose2d());
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        poseEstimator.update(odoAngle(), odoPosition());
        limelight.odometryUpdate(poseEstimator);

        if (driveState == driveType.CROSS) {
            //set to cross - done in inputupdate
            this.swerveSignal = swerveHelper.setCross();
            drive();
        }
        if (driveState == driveType.TELEOP) {
            if (rotLocked){
                //if rotation tracking, replace rotational joystick value with controller generated one
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            }
            this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
            SmartDashboard.putNumber("FR signal", swerveSignal.getSpeed(0));
            drive();
        }
        if (driveState == driveType.AUTO) {
            //get controller generated rotation value
            // rotSpeed = Math.max(-0.2, Math.min(0.2, swerveHelper.getRotControl(pathTarget, getGyroAngle())));
            rotSpeed = swerveHelper.getRotControl(pathTarget, getGyroAngle());
            //ensure rotation is never more than 0.2 to prevent normalization of translation from occuring
            
            //update where the robot is, to determine error in path
            this.swerveSignal = swerveHelper.setAuto(swerveHelper.getAutoPower(pathVel, pathAccel), pathHeading, rotSpeed,getGyroAngle(),pathXOffset, pathYOffset);
            drive();        
        }
        //Turn Robot Toward Speaker
        if(driveState == driveType.SPEAKER){
            rotTarget = getAngleToSpeaker();
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            this.swerveSignal = swerveHelper.setDrive(0, 0, rotSpeed, getGyroAngle());
            SmartDashboard.putNumber("FR signal", swerveSignal.getSpeed(0));
            drive();
            

        }

        if(driveState == driveType.AMP && isInAmpRadius()){
            double xSpeed = 0;
            double ySpeed = 0;

            ySpeed = (DriveConstants.AMP_Y-poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
            
            if(poseEstimator.getEstimatedPosition().getX() < lc.ALLIANCE_LENGTH){
                xSpeed = ((lc.ALLIANCE_LENGTH - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                rotSpeed = swerveHelper.getRotControl(90, getGyroAngle());
            }else if(poseEstimator.getEstimatedPosition().getX() > (lc.ALLIANCE_LENGTH + lc.CENTER_FIELD_LENGTH)){
                xSpeed = ((DriveConstants.FIELD_LENGTH - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                rotSpeed = swerveHelper.getRotControl(90, getGyroAngle());
            }
            this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
            SmartDashboard.putNumber("FR signal", swerveSignal.getSpeed(0));
            drive();
        }
        if(driveState == driveType.STAGE){
            double ySpeed = 0;
            double xSpeed = 0;
            String closestTag = getClosestChain();

            if(station.orElse(null).equals(Alliance.Blue)){
                if(closestTag.equals("tag16")) {
                   ySpeed = (lc.Chain16Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain16Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(-135, getGyroAngle());
                }else if(closestTag.equals("tag15")){
                   ySpeed = (lc.Chain15Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain15Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(135, getGyroAngle());
                }else if(closestTag.equals("tag14")){
                   ySpeed = (lc.Chain14Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain14Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(0, getGyroAngle());
                }
            }else if(station.orElse(null).equals(Alliance.Red)){
                if(closestTag.equals("tag13")) {
                   ySpeed = (lc.Chain13Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain13Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(180, getGyroAngle());
                }else if(closestTag.equals("tag12")){
                   ySpeed = (lc.Chain12Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain12Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(45, getGyroAngle());
                }else if(closestTag.equals("tag11")){
                   ySpeed = (lc.Chain11Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                   xSpeed = ((lc.Chain11Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                   rotSpeed = swerveHelper.getRotControl(-45, getGyroAngle());
                }
            }    
        }
        
        this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
        SmartDashboard.putNumber("FR signal", swerveSignal.getSpeed(0));
        drive();

        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X speed", xSpeed);
        SmartDashboard.putNumber("Y speed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putBoolean("rotLocked", rotLocked);
        SmartDashboard.putNumber("Auto velocity", pathVel);
        SmartDashboard.putNumber("Auto translate direction", pathHeading);
        SmartDashboard.putNumber("Auto rotation target", pathTarget);
}
    
    @Override
    public void resetState() {
        xSpeed = 0;
        ySpeed = 0;
        rotSpeed = 0;
        setToTeleop();
        rotLocked = false;
        rotTarget = 0.0;
        pathVel = 0.0;
        pathHeading = 0.0;
        pathAccel = 0.0;
        pathTarget = 0.0;

        isFieldCentric = true;
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }

    /** resets the drive encoders on each module */
    public void resetDriveEncoders() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetDriveEncoders();
        }
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = driveType.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(false);
        }
        rotSpeed = 0;
        xSpeed = 0;
        ySpeed = 0;
        pathHeading = 0;
        pathVel = 0;
        pathAccel = 0;
        rotLocked = false;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = driveType.AUTO;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
    }

    /**drives the robot at the current swerveSignal, and displays information for each swerve module */
    private void drive() {
        if (driveState == driveType.CROSS) {
            for (int i = 0; i < modules.length; i++) {
                modules[i].runCross(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
                modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
            }
        }
        else {
            for (int i = 0; i < modules.length; i++) {
                modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
                modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
            }
        }
    }

    /**sets autonomous values from the path data file */
    public void setAutoValues(double velocity, double heading, double accel, double xOffset, double yOffset) {
        pathVel = velocity;
        pathHeading = heading;
        pathAccel = accel;
        pathXOffset = xOffset;
        pathYOffset = yOffset;
    }

    /**sets the autonomous heading controller to a new target */
    public void setAutoHeading(double headingTarget) {
        pathTarget = headingTarget;
    }

    /**
     * Resets the gyro, and sets it the input number of degrees
     * Used for starting the match at a non-0 angle
     * @param degrees the current value the gyro should read
     */
    public void setGyro(double degrees) {
        resetState();
        setToAuto();
        gyro.setYaw(degrees);
    }

    public double getGyroAngle() {
        if (!isFieldCentric) return 0;
        return (359.99 - gyro.getYaw()+360)%360;
    }  
    public Rotation2d odoAngle(){
        return new Rotation2d(Math.toRadians(360-getGyroAngle()));
    }
    public SwerveModulePosition[] odoPosition(){
        return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    }
    public void setOdo(Pose2d pos){
        this.poseEstimator.resetPosition(odoAngle(), odoPosition(), pos);
        autoTimer.start();
    }
    public Pose2d returnPose(){
        return poseEstimator.getEstimatedPosition();
    }
}
