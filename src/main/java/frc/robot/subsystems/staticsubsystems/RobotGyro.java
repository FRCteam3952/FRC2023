package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * Wrapper around gyro
 */

public class RobotGyro {
    private static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    static {
        gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
        gyro.calibrate();
    }

    /**
     * make sure this class is instantiated properly by poking it
     */
    public static void poke() {
        System.out.println("RobotGyro init");
    }

    private static double angleAdjust = 0;

    public static double getGyroAngleDegrees() {
        return gyro.getAngle() + angleAdjust;
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
    public static void robotCalibrate(){
        gyro.calibrate();
    }
}
