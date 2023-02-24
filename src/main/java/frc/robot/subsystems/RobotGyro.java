package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * Wrapper around gyro
 */

public class RobotGyro extends SubsystemBase {
    private static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    static {
        gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
        gyro.calibrate();
    }

    /**
     * make sure this class is instantiated properly by poking it
     */
    public static void poke() {
        System.out.println("Gyro init");
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

    @Override
    public void periodic() {
        // System.out.println(getGyroAngle());
        // resetGyroAngle();
    }

    @Override
    public void simulationPeriodic() {

    }
}