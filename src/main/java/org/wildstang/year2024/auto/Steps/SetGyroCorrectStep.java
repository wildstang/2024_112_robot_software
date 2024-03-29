package org.wildstang.year2024.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.year2024.subsystems.swerve.SwerveDrive;

public class SetGyroCorrectStep extends AutoStep {

    private SwerveDrive m_drive;

    /** sets the gyro value to be the given argument value
     * @param heading value you want the gyro to currently read
     * @param drive the swerveDrive subsystem
     */
    public SetGyroCorrectStep(SwerveDrive drive) {
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.setGyro(m_drive.getPosTheta());
        m_drive.setToAuto();
    }

    @Override
    public void update() {
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Gyro";
    }

}
