package org.wildstang.year2024.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
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

    private AnalogInput leftStickX;  // translation joystick x
    private AnalogInput leftStickY;  // translation joystick y
    private AnalogInput rightStickX;  // rot joystick
    private AnalogInput leftTrigger, rightTrigger;  //speed derate, thrust 
    private DigitalInput leftBumper, rightBumper;  // intake, shoot
    private DigitalInput select;  // gyro reset
    private DigitalInput start;  // defense mode
    private DigitalInput faceUp;  // rotation lock 0 degrees
    private DigitalInput faceRight;  // rotation lock 90 degrees
    private DigitalInput faceLeft;  // rotation lock 270 degrees
    private DigitalInput faceDown;  // rotation lock 180 degrees

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

    private WsVision pvCam;
    private double targetYaw;

    public enum driveType {TELEOP, AUTO, CROSS, SPEAKER, AMP, STAGE, VISION};
    public driveType driveState;
    private Boolean isBlueAlliance = null;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);
    }

    public void initInputs() {
        pvCam = (WsVision) Core.getSubsystemManager().getSubsystem("Ws Vision");

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
        poseEstimator = new SwerveDrivePoseEstimator(new SwerveDriveKinematics(new Translation2d(DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2),
            new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2)), odoAngle(), odoPosition(), new Pose2d());
    }

    @Override
    public void inputUpdate(Input source) {
        //determine if we are in cross or teleop
        if (driveState != driveType.AUTO && start.getValue()) {
            driveState = driveType.CROSS;
            for (int i = 0; i < modules.length; i++) {
                modules[i].setDriveBrake(true);
            }
            this.swerveSignal = new SwerveSignal(new double[]{0, 0, 0, 0 }, swerveHelper.setCross().getAngles());
        }
        else if (driveState == driveType.CROSS || driveState == driveType.AUTO) {
            driveState = driveType.TELEOP;
        }
        if(source == leftBumper){
            if(leftBumper.getValue()) driveState = driveType.VISION;
            else driveState = driveType.TELEOP;
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
    public void update() {
        if (isBlueAlliance == null){
            if (DriverStation.getAlliance().isPresent()){
                isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
            }
        }
        
        poseEstimator.update(odoAngle(), odoPosition());
        targetYaw = pvCam.getYaw();
        pvCam.odometryUpdate(poseEstimator);

        switch (driveState) {
            case CROSS:
                //set to cross - done in inputupdate
                this.swerveSignal = swerveHelper.setCross();
                drive();
                break;

            case TELEOP:
                if (rotLocked){
                    //if rotation tracking, replace rotational joystick value with controller generated one
                    rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;

            case AUTO:
                //get controller generated rotation value
                // rotSpeed = Math.max(-0.2, Math.min(0.2, swerveHelper.getRotControl(pathTarget, getGyroAngle())));
                rotSpeed = swerveHelper.getRotControl(pathTarget, getGyroAngle());
                //ensure rotation is never more than 0.2 to prevent normalization of translation from occuring
                
                //update where the robot is, to determine error in path
                this.swerveSignal = swerveHelper.setAuto(swerveHelper.getAutoPower(pathVel, pathAccel), pathHeading, rotSpeed,getGyroAngle(),pathXOffset, pathYOffset);
                drive();  
                break;

            case SPEAKER:
                //Turn Robot Toward Speaker
                rotTarget = getAngleToSpeaker();
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                this.swerveSignal = swerveHelper.setDrive(0, 0, rotSpeed, getGyroAngle());
                drive();
                break;

            case AMP:
                ySpeed = (FieldConstants.AMP_Y - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                if (isBlueAlliance) {
                    xSpeed = (FieldConstants.BLUE_AMP_X - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P;
                    rotSpeed = swerveHelper.getRotControl(270, getGyroAngle());
                } else {
                    xSpeed = (FieldConstants.RED_AMP_X - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P;
                    rotSpeed = swerveHelper.getRotControl(270, getGyroAngle());
                }
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;

            case STAGE:
                String closestTag = getClosestChain();

                if(isBlueAlliance){
                    if(closestTag.equals("tag16")) {
                        ySpeed = (FieldConstants.Chain16Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain16Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(-135, getGyroAngle());
                    }else if(closestTag.equals("tag15")){
                        ySpeed = (FieldConstants.Chain15Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain15Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(135, getGyroAngle());
                    }else if(closestTag.equals("tag14")){
                        ySpeed = (FieldConstants.Chain14Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain14Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(0, getGyroAngle());
                    }
                } else {
                    if(closestTag.equals("tag13")) {
                        ySpeed = (FieldConstants.Chain13Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain13Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(180, getGyroAngle());
                    }else if(closestTag.equals("tag12")){
                        ySpeed = (FieldConstants.Chain12Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain12Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(45, getGyroAngle());
                    }else if(closestTag.equals("tag11")){
                        ySpeed = (FieldConstants.Chain11Midpoint[1] - poseEstimator.getEstimatedPosition().getY()) * DriveConstants.POS_P;
                        xSpeed = ((FieldConstants.Chain11Midpoint[0] - poseEstimator.getEstimatedPosition().getX()) * DriveConstants.POS_P);
                        rotSpeed = swerveHelper.getRotControl(-45, getGyroAngle());
                    }
                }  
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;

            case VISION:
                if (!Double.isNaN(targetYaw)){
                    rotSpeed = pvCam.getYaw() * .005;
                    this.swerveSignal = swerveHelper.setDrive(0, 0, rotSpeed, getGyroAngle());
                    drive();
                }
                break;
        }

        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X speed", xSpeed);
        SmartDashboard.putNumber("Y speed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putBoolean("rotLocked", rotLocked);
        SmartDashboard.putNumber("Auto velocity", pathVel);
        SmartDashboard.putNumber("Auto translate direction", pathHeading);
        SmartDashboard.putNumber("Auto rotation target", pathTarget);
        SmartDashboard.putNumber("target yaw", targetYaw);
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

    public String getClosestChain(){
        double robotX = getPosX();
        double robotY = getPosY();
        if(isBlueAlliance){
            double distanceFromTag16 = distanceFrom(robotX,FieldConstants.tag16[0],robotY, FieldConstants.tag16[1]);
            double distanceFromTag15 = distanceFrom(robotX,FieldConstants.tag15[0],robotY, FieldConstants.tag15[1]);
            double distanceFromTag14 = distanceFrom(robotX,FieldConstants.tag14[0],robotY, FieldConstants.tag14[1]);
            double smallest = Math.min(Math.min(distanceFromTag16, distanceFromTag15), distanceFromTag14);

            if(smallest == distanceFromTag16){
                return "tag16";
            }else if(smallest == distanceFromTag15){
                return "tag15";
            }else if(smallest == distanceFromTag14){
                return "tag14";
            }
        } else {
            double distanceFromTag13 = distanceFrom(robotX, FieldConstants.tag13[0], robotY, FieldConstants.tag13[1]);
            double distanceFromTag12 = distanceFrom(robotX, FieldConstants.tag12[0], robotY, FieldConstants.tag12[1]);
            double distanceFromTag11 = distanceFrom(robotX, FieldConstants.tag11[0], robotY, FieldConstants.tag11[1]);
            double smallest = Math.min(Math.min(distanceFromTag13, distanceFromTag12), distanceFromTag11);

            if(smallest == distanceFromTag11){
                return "tag11";
            }else if(smallest == distanceFromTag12){
                return "tag12";
            }else if(smallest == distanceFromTag13){
                return "tag13";
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

        if(isBlueAlliance){
            xPosition = FieldConstants.BLUE_SPEAKER_X - poseEstimator.getEstimatedPosition().getX();
            yPosition = -poseEstimator.getEstimatedPosition().getY();

            angleToSpeaker = Math.atan(xPosition/yPosition);
            
            
        }else {
            xPosition = poseEstimator.getEstimatedPosition().getX() - FieldConstants.RED_SPEAKER_X;
            yPosition = -poseEstimator.getEstimatedPosition().getY();
            
            angleToSpeaker = Math.atan(xPosition/yPosition);
        }
        return angleToSpeaker;
    }

     //blue is true and red is falase
     public double getDistanceFromSpeaker(){
        double xDifference = 0;
        Pose2d fieldPose = returnPose();
        double yDifference = FieldConstants.SPEAKER_Y - fieldPose.getY();
        if(isBlueAlliance){
            xDifference = FieldConstants.BLUE_SPEAKER_X - fieldPose.getX();
        }
        else {
            xDifference = FieldConstants.RED_SPEAKER_X - fieldPose.getX();
        }
        return Math.sqrt(yDifference * yDifference + xDifference * xDifference);

    }
    public double getDistanceFromAmp(){
        double xDiff = 0;
        Pose2d fieldPose = returnPose();
        double yDiff = FieldConstants.AMP_Y - fieldPose.getY();
        if(isBlueAlliance){
            xDiff = FieldConstants.BLUE_AMP_X - fieldPose.getX();
        }
        else {
            xDiff = FieldConstants.RED_SPEAKER_X - fieldPose.getX();
        }
        double distance = Math.sqrt(yDiff * yDiff + xDiff * xDiff);
        return distance;

    }

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
    public void selfTest() {
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

    public Boolean isAtTarget(){
        return (Math.abs(targetYaw) < 4 || Double.isNaN(targetYaw));
    }
}
