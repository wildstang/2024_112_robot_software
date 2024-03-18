package org.wildstang.framework.subsystems.swerve;

import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class SwerveDriveTemplate implements Subsystem{
    
    public abstract void setAutoValues(double xVel, double yVel, double angVel, double xPos, double yPos, double heading);

    public abstract void resetDriveEncoders();

    public abstract void setAutoHeading(double headingTarget);

    public abstract void setGyro(double degrees);

    public abstract void setToAuto();

    public abstract void setToTeleop();

    public abstract Pose2d returnPose();

}
