package org.wildstang.year2024.subsystems.swerve;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.CANConstants;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.subsystems.targeting.WsVision;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Translation2dProto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics.ProtobufSwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JennySwerveDrive extends SwerveDriveTemplate {

    private AnalogInput leftStickX;
    private AnalogInput leftStickY;
    private AnalogInput rightStickX;
    private DigitalInput select;

    SwerveDriveKinematics kinematics;
    ChassisSpeeds chassisSpeeds;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    private SwerveModule modules[];
    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    private WsVision pvCamera;
    

    private double xOutput;
    private double yOutput;
    private double rotOutput;
    private boolean isBlueAlliance;
    private double robotVelMag; // speed the robot is traveling at 
    private double robotVelTheta; //angle the velocity vector of the drive base is at
    private Pose2d curPose;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    



    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
    }

    private void initInputs(){
        leftStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_X);
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        rightStickX.addInputListener(this);
        select =  (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_SELECT);
        select.addInputListener(this);
    }
    private void initOutputs(){

        //initialize array of swerve motors
        modules = new SwerveModule[]{ new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE1), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE1), DriveConstants.FRONT_LEFT_OFFSET), 
                new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE2), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE2), DriveConstants.FRONT_RIGHT_OFFSET),
            new SwerveModule((WsSpark)Core.getOutputManager().getOutput(WsOutputs.DRIVE3), 
            (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE3), DriveConstants.REAR_LEFT_OFFSET),
        new SwerveModule((WsSpark)Core.getOutputManager().getOutput(WsOutputs.DRIVE4),
        (WsSpark)Core.getOutputManager().getOutput(WsOutputs.ANGLE4),DriveConstants.REAR_RIGHT_OFFSET)};

        //create a new swerve drive kinematics object
        kinematics = new SwerveDriveKinematics(new Translation2d(
          DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2),
          new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2),
          new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2),
          new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2)
          );
        //initalize swerve signal object starting at angle + speeds of 0 being sent
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0,0.0,0.0,0.0});
        //initalize odometry object
        odometry = new SwerveDriveOdometry(kinematics, returnGyroAngle(), getAllModulePositions(), new Pose2d());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, returnGyroAngle(), getAllModulePositions(), new Pose2d());
        pvCamera = (WsVision) Core.getSubsystemManager().getSubsystem("Ws Vision");
        }

    
    @Override
    public void inputUpdate(Input source) {

        if(source == select && select.getValue()){
            if(isBlueAlliance){
                gyro.setYaw(0.0);
                rotOutput = 0.0;
            }
            else{
                gyro.setYaw(Math.PI * RAD_TO_DEG);
                rotOutput = Math.PI;
            }
        }

        xOutput = swerveHelper.scaleDeadband(leftStickY.getValue(),DriveConstants.DEADBAND);
        yOutput = swerveHelper.scaleDeadband(-leftStickX.getValue(), DriveConstants.DEADBAND);
        rotOutput = swerveHelper.scaleDeadband(-rightStickX.getValue(), DriveConstants.DEADBAND);
        

        //red alliance
        if(!isBlueAlliance){
            xOutput *= -1;
            yOutput *= -1;
            rotOutput =  (rotOutput + Math.PI) % 2*Math.PI;
        }


    }


    @Override
    public void update() {
        odometry.update(returnGyroAngle(), getAllModulePositions());
        for(int i = 0; i < modules.length; i++ ){
            moduleStates[i] = modules[i].getModuleState();
        }
        poseEstimator.update(returnGyroAngle(),getAllModulePositions());
        pvCamera.odometryUpdate(poseEstimator);
        curPose = poseEstimator.getEstimatedPosition();


        //field centric velocity of drive base
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), Rotation2d.fromRadians(getGyroAngle()));
        robotVelMag = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        robotVelTheta = Math.atan2(chassisSpeeds.vyMetersPerSecond,chassisSpeeds.vxMetersPerSecond);



        rotOutput = Math.min(Math.max(rotOutput, -1), 1);
        xOutput = Math.min(Math.max(xOutput, -1), 1);
        yOutput = Math.min(Math.max(yOutput, -1), 1);

        swerveSignal = swerveHelper.setDrive(xOutput,yOutput, rotOutput, getGyroAngle());
        drive();


        putSmartDashboard();
    }


    private void drive(){
        for(int i = 0; i < modules.length; i++){
            modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
            modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
        }
    }

    private void putSmartDashboard(){
        SmartDashboard.putNumber("robot vel mag", robotVelMag);
        SmartDashboard.putNumber("robot vel theta", robotVelTheta);
    }


    @Override
    public void resetState() {
        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }
    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }
     @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void setAutoValues(double xVel, double yVel, double angVel, double xPos, double yPos, double heading) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAutoValues'");
    }

    @Override
    public void resetDriveEncoders() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetDriveEncoders'");
    }

    @Override
    public void setAutoHeading(double headingTarget) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAutoHeading'");
    }

    @Override
    public void setGyro(double degrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setGyro'");
    }

    @Override
    public void setToAuto() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setToAuto'");
    }

    @Override
    public void setToTeleop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setToTeleop'");
    }

    @Override
    public Pose2d returnPose() {
        return odometry.getPoseMeters();
    }
    
    //methood that returns an array of swerve module positions
    //aka the linnear distance traveled and current angle
    public SwerveModulePosition[] getAllModulePositions(){
        return new SwerveModulePosition[]{modules[0].odoPosition(),modules[1].odoPosition(),modules[2].odoPosition(),modules[3].odoPosition()};
    }

    //gets gyro angle in radians, normalizes it to be in range as well as making sure the result is positive
    public double getGyroAngle(){
        return (((gyro.getYaw() * DEG_TO_RAD) % 2*Math.PI)+(2*Math.PI)) % 2*Math.PI;
    }

    //returns the gyro angle as a rotation2d object
    public Rotation2d returnGyroAngle(){
        return new Rotation2d(getGyroAngle());
    }
    
}
