package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * Wrapper around gyro
 */

public class RobotGyro {
    private static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    static {
        gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
        gyro.calibrate();
        gyro.reset();
    }

    /**
     * make sure this class is instantiated properly by poking it
     */
    public static void poke() {
        System.out.println("RobotGyro init");
    }

    private static double angleAdjust = 0;

    public static Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(gyro.getAngle() + angleAdjust));
    }

    public static double getGyroAngleDegreesYaw() {
        return gyro.getAngle() + angleAdjust;
    }

    public static double getGyroAngleDegreesRoll() {
        return gyro.getXComplementaryAngle() + angleAdjust;
    }

    public static double getGyroAngleDegreesPitch() {
        return gyro.getYComplementaryAngle() + angleAdjust;
    }


    public static void setGyroAxis(ADIS16470_IMU.IMUAxis axis) {
        gyro.setYawAxis(axis);
    }

    public static void resetGyroAngle() {
        gyro.reset();
        angleAdjust = 0;
    }

    public static void setGyroAngle(double angle) {
        resetGyroAngle();
        angleAdjust = angle;
    }

    public static void robotCalibrate() {
        gyro.calibrate();
    }

    public static double getGyroGeneralAcceleration() {
        return gyro.getAccelX() + gyro.getAccelY();
    }
}
