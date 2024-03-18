package org.wildstang.year2024.subsystems.swerve;

public class WsSwerveHelper {

    private SwerveSignal swerveSignal;
    private double rotMag;
    private double baseV;
    private double[] xCoords = new double[]{0.0, 0.0, 0.0, 0.0};
    private double[] yCoords = new double[]{0.0, 0.0, 0.0, 0.0};
    private double rotErr;
    private double rotPID;

    /** sets the robot in the immobile "cross" defensive position
     * 
     * @return driveSignal for cross, with 0 magnitude and crossed directions
     */
    public SwerveSignal setCross() {
        return new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{135.0, 45.0, 45.0, 135.0});
    }

    /**sets the robot to drive normally as a swerve
     * 
     * @param i_tx the translation joystick x value
     * @param i_ty the translation joystick y value
     * @param i_rot the rotation joystick value
     * @param i_gyro the gyro value, field centric, in bearing degrees
     * @return driveSignal for normal driving, normalized
     */
    public SwerveSignal setDrive(double i_tx, double i_ty, double i_rot, double i_gyro) {
        //magnitude of rotation vector
        rotMag = i_rot;
        //angle of front left rotation vector
        baseV = Math.atan(DriveConstants.ROBOT_LENGTH / DriveConstants.ROBOT_WIDTH);

        //find the translational vectors rotated to account for the gyro
        double xTrans = i_tx * Math.cos(i_gyro) + i_ty * Math.sin(i_gyro);
        double yTrans = - i_tx * Math.sin(i_gyro) + i_ty * Math.cos(i_gyro);

        //account for slight second order skew due to rotation and translation at the same time
        xTrans += Math.cos(Math.atan2(xTrans,yTrans)) * i_rot * DriveConstants.ROT_CORRECTION_FACTOR * Math.hypot(i_tx, i_ty);
        yTrans += -Math.sin(Math.atan2(xTrans,yTrans)) * i_rot * DriveConstants.ROT_CORRECTION_FACTOR * Math.hypot(i_tx, i_ty);

        //cartesian vector addition of translation and rotation vectors
        //note rotation vector angle advances in the cos -> sin -> -cos -> -sin fashion
        xCoords = new double[]{xTrans + rotMag * Math.cos(baseV), xTrans + rotMag*Math.sin(baseV), xTrans - rotMag * Math.sin(baseV), xTrans - rotMag*Math.cos(baseV)}; 
        yCoords = new double[]{yTrans + rotMag * Math.sin(baseV), yTrans - rotMag*Math.cos(baseV), yTrans + rotMag * Math.cos(baseV), yTrans - rotMag*Math.sin(baseV)};

        //create drivesignal, with magnitudes and directions of x and y
        swerveSignal = new SwerveSignal(new double[]{Math.hypot(xCoords[0], yCoords[0]), Math.hypot(xCoords[1], yCoords[1]), Math.hypot(xCoords[2], yCoords[2]), Math.hypot(xCoords[3], yCoords[3])}, 
            new double[]{getDirection(xCoords[0], yCoords[0]), getDirection(xCoords[1], yCoords[1]), getDirection(xCoords[2], yCoords[2]), getDirection(xCoords[3], yCoords[3])});
        swerveSignal.normalize();
        return swerveSignal;
        //slew rate limit
        // if (swerveSignal.isNotZeroed()) {
        //     return swerveSignal;
        // }
        // else {
        //     return this.setDrive(i_tx, i_ty, i_rot * 0.8, i_gyro);
        // }
    }

    /**automatically creates a rotational joystick value to rotate the robot towards a specific target
     * 
     * @param i_target target direction for the robot to face, field centric, radians
     * @param i_gyro the gyro value, field centric, in radians
     * @return double that indicates what the rotational joystick value should be
     */
    public double getRotControl(double i_target, double i_gyro) {
        rotErr = i_target - i_gyro;
        if (rotErr > Math.PI) {
            rotPID = (rotErr - Math.PI * 2.0) * DriveConstants.ROT_P;  // if error is greater than pi, it is faster to spin cw
        }
        else {
            rotPID = rotErr * DriveConstants.ROT_P;  // otherwise spin ccw
        }
        return Math.max(Math.min(rotPID, 1.0), -1.0);  // saturate rotation control to range [-1.0, 1.0]
    }

    /**determines the translational magnitude of the robot in autonomous
     * 
     * @param pathVel path data for velocity of the robot, inches
     * @return double for magnitude of translational vector
     */
    public double getAutoPower(double pathVel, double pathAccel) {
        if (pathVel == 0) return 0;
        double guess = pathVel * DriveConstants.DRIVE_F_V + DriveConstants.DRIVE_F_K + pathAccel * DriveConstants.DRIVE_F_I;
        return -(guess);
    }

    /**x,y inputs are cartesian, angle values are in bearing, returns 0 - 360 */
    public double getDirection(double x, double y) {
        double measurement = Math.toDegrees(Math.atan2(x,y));//returns angle in bearing form
        if (measurement < 0) {
            measurement = 360 + measurement;
        }
        else if (measurement >= 360) {
            measurement = measurement - 360;
        }
        return measurement;
    }
    
    public double scaleDeadband(double input, double deadband){
        if (Math.abs(input) < deadband) return 0.0;
        return Math.signum(input)*((Math.abs(input) - deadband) / (1.0 - deadband));
    }
    
}
