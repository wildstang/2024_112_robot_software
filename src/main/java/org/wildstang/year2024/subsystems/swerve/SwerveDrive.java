package org.wildstang.year2024.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import org.photonvision.PhotonUtils;
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
    private DigitalInput start;  // 
    private DigitalInput faceUp;  // rotation lock 0 degrees
    private DigitalInput faceRight;  // rotation lock 90 degrees
    private DigitalInput faceLeft;  // rotation lock 270 degrees
    private DigitalInput faceDown;  // rotation lock 180 degrees
    private DigitalInput dpadUp;

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
    // private double targetYaw;

    public enum driveType {TELEOP, AUTO, SPEAKER, AMP, STAGE};
    public driveType driveState;
    private Boolean isBlueAlliance = null;
    private Pose2d curPose, goalPose;

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
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
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
        driveState = driveType.TELEOP;
        //determine if we are in cross or teleop
        // if (source == start && start.getValue()) {
        //     driveState = driveType.CROSS;
        //     for (int i = 0; i < modules.length; i++) {
        //         modules[i].setDriveBrake(true);
        //     }
        //     this.swerveSignal = new SwerveSignal(new double[]{0, 0, 0, 0 }, swerveHelper.setCross().getAngles());
        // }
        if (source == leftBumper && leftBumper.getValue()) driveState = driveType.SPEAKER;
        if (source == dpadUp && dpadUp.getValue()) driveState = driveType.AMP;
        if (source == start && start.getValue()){
            driveState = driveType.STAGE;
            goalPose = getClosestChain();
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
            }
        }

        //get rotational joystick
        rotSpeed = swerveHelper.scaleDeadband(rightStickX.getValue(), DriveConstants.DEADBAND);
        rotSpeed *= Math.abs(rotSpeed);
        rotSpeed *= DriveConstants.ROTATION_SPEED;
        //if the rotational joystick is being used, the robot should not be auto tracking heading
        if (rotSpeed == 0) {
            rotTarget = getGyroAngle();
            rotLocked = true;
        } else {
            rotLocked = false;
        }
        
        //assign thrust
        thrustValue = 1 - DriveConstants.DRIVE_THRUST + DriveConstants.DRIVE_THRUST * Math.abs(rightTrigger.getValue());
        derateValue = (DriveConstants.DRIVE_DERATE * Math.abs(leftTrigger.getValue()) + 1);
        xSpeed *= thrustValue/derateValue;
        ySpeed *= thrustValue/derateValue;
        rotSpeed *= thrustValue/derateValue;
    }
    @Override
    public void update() {
        if (isBlueAlliance == null){
            if (DriverStation.getAlliance().isPresent()){
                isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
            }
        }
        
        poseEstimator.update(odoAngle(), odoPosition());
        // targetYaw = pvCam.getYaw();
        pvCam.odometryUpdate(poseEstimator);

        switch (driveState) {
            case TELEOP:
                if (rotLocked){
                    //if rotation tracking, replace rotational joystick value with controller generated one
                    rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;
            case AUTO:
                rotSpeed = swerveHelper.getRotControl(pathTarget, getGyroAngle());  // get controller generated rotation value
                // rotSpeed = Math.max(-0.2, Math.min(0.2, swerveHelper.getRotControl(pathTarget, getGyroAngle())));  // ensure rotation is never more than 0.2 to prevent normalization of translation from occuring
                
                //update where the robot is, to determine error in path
                this.swerveSignal = swerveHelper.setAuto(swerveHelper.getAutoPower(pathVel, pathAccel), pathHeading, rotSpeed, getGyroAngle(), pathXOffset, pathYOffset);
                drive();  
                break;
            case SPEAKER:
                //Turn Robot Toward Speaker
                rotTarget = getAngleToSpeaker();
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;
            case AMP:
                if (isBlueAlliance) {
                    goalPose = FieldConstants.BLUE_AMP;
                } else {
                    goalPose = FieldConstants.RED_AMP;
                }
                curPose = poseEstimator.getEstimatedPosition();
                xSpeed = (goalPose.getX() - curPose.getX()) * DriveConstants.POS_P;
                ySpeed = (goalPose.getY() - curPose.getY()) * DriveConstants.POS_P;
                rotSpeed = swerveHelper.getRotControl(goalPose.getRotation().getDegrees(), getGyroAngle());
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
                break;
            case STAGE:
                curPose = poseEstimator.getEstimatedPosition();
                xSpeed = (goalPose.getX() - curPose.getX()) * DriveConstants.POS_P;
                ySpeed = (goalPose.getY() - curPose.getY()) * DriveConstants.POS_P;
                rotSpeed = swerveHelper.getRotControl(goalPose.getRotation().getDegrees(), getGyroAngle());
                this.swerveSignal = swerveHelper.setDrive(xSpeed, ySpeed, rotSpeed, getGyroAngle());
                drive();
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
        SmartDashboard.putBoolean("drive at target", isAtTarget());
        // SmartDashboard.putNumber("target yaw", targetYaw);
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

    public Pose2d getClosestChain(){
        Pose2d curPose = poseEstimator.getEstimatedPosition();
        Pose2d returnPose;
        PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain14);
        if(isBlueAlliance){
            returnPose = FieldConstants.Chain14;
            double dist = PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain14);
            if (PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain15) < dist) {
                returnPose = FieldConstants.Chain15;
                dist = PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain15);
            }
            if (PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain16) < dist) {
                returnPose = FieldConstants.Chain16;
            }
        } else {
            returnPose = FieldConstants.Chain11;
            double dist = PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain11);
            if (PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain12) < dist) {
                returnPose = FieldConstants.Chain12;
                dist = PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain12);
            }
            if (PhotonUtils.getDistanceToPose(curPose, FieldConstants.Chain13) < dist) {
                returnPose = FieldConstants.Chain13;
            }
        }
        return returnPose;
    }

     // Get x Pos and y Pos and calulate angle of turn needed to line up with speaker
     public double getAngleToSpeaker(){
        if (isBlueAlliance) {
            return PhotonUtils.getYawToPose(poseEstimator.getEstimatedPosition(), FieldConstants.BLUE_SPEAKER).getDegrees();
        } else {
            return PhotonUtils.getYawToPose(poseEstimator.getEstimatedPosition(), FieldConstants.RED_SPEAKER).getDegrees();
        }
    }

     //blue is true and red is falase
     public double getDistanceFromSpeaker(){
        if (isBlueAlliance) {
            return PhotonUtils.getDistanceToPose(poseEstimator.getEstimatedPosition(), FieldConstants.BLUE_SPEAKER);
        } else {
            return PhotonUtils.getDistanceToPose(poseEstimator.getEstimatedPosition(), FieldConstants.RED_SPEAKER);
        }
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
        for (int i = 0; i < modules.length; i++) {
            modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
            modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
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
    public void setPose(Pose2d pos){
        this.poseEstimator.resetPosition(odoAngle(), odoPosition(), pos);
        autoTimer.start();
    }
    public Pose2d returnPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public Boolean isAtTarget(){
        // return PhotonUtils.getDistanceToPose(poseEstimator.getEstimatedPosition(), goalPose) < DriveConstants.POS_DB;
        // return (Math.abs(targetYaw) < 4 || Double.isNaN(targetYaw));
        return true;
    }
}
