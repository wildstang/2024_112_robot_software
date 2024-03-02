package org.wildstang.year2024.subsystems.notepath;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2024.robot.WsInputs;
import org.wildstang.year2024.robot.WsOutputs;
import org.wildstang.year2024.robot.WsSubsystems;
import org.wildstang.year2024.subsystems.ampHood.AmpHood;
import org.wildstang.year2024.subsystems.armPivot.ArmPivot;
import org.wildstang.year2024.subsystems.shooter.Shooter;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Notepath implements Subsystem{
    private WsSpark intakeMotor;
    private WsSpark feedMotor;
    private double intakeMotorSpeed = 0;
    private double feedMotorSpeed = 0;
    private DigitalInput rightBumper, leftBumper;
    private DigitalInput dpadDown, dpadUp;
    private ArmPivot armPivot;
    private Shooter shooter;
    private AmpHood ampHood;
    private enum feedType {SPEAKER, AMP, INTAKE, OUTTAKE, OFF};
    private feedType feedState;
    private SwerveDrive drive;

    @Override
    public void inputUpdate(Input source) {
        if (leftBumper.getValue()){
            feedState = feedType.SPEAKER;
        } else if (dpadUp.getValue()) {
            feedState = feedType.AMP;
        } else if (rightBumper.getValue()){
            feedState = feedType.INTAKE;
        } else if (dpadDown.getValue()){
            feedState = feedType.OUTTAKE;
        } else {
            feedState = feedType.OFF;
        }

        
        
    }

    @Override
    public void init() {
        intakeMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.INTAKE);
        feedMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.FEED);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);

        armPivot = (ArmPivot) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARM_PIVOT);
        shooter = (Shooter) Core.getSubsystemManager().getSubsystem(WsSubsystems.SHOOTER);
        ampHood = (AmpHood) Core.getSubsystemManager().getSubsystem(WsSubsystems.AMP_HOOD);
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        resetState();

    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        switch (feedState) {
            case SPEAKER:
            case AMP:
                if (armPivot.isAtTarget() && shooter.isAtTarget() && ampHood.isAtTarget() && drive.isAtTarget()) {
                    feedMotorSpeed = 0.5;
                } else {
                    feedMotorSpeed = 0.0;
                }
                intakeMotorSpeed = 0.0;
                break;

            case INTAKE:
                feedMotorSpeed = 0.15;
                intakeMotorSpeed = 0.8;
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

        feedMotor.setSpeed(feedMotorSpeed);
        intakeMotor.setSpeed(intakeMotorSpeed);

        SmartDashboard.putNumber("feed speed", feedMotorSpeed);
    }

    @Override
    public void resetState() {
        feedMotorSpeed = 0.0;
        intakeMotorSpeed = 0.0;
        feedState = feedType.OFF;
    }

    @Override
    public String getName() {
        //make sure to put a string name for the system here
        return "notepath";
    }


}