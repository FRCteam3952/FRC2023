package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * Wrapper around gyro
 */

public class RobotGyro extends SubsystemBase {
    private static final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private static final int DRIFT_MEASUREMENT_SAMPLES = 20; //specifies the number of measurements to take during each interval CHANGED IF NEEDED
    private static final int DRIFT_MEASUREMENT_INTERVAL_MS = 25; //specifies the duration of each measurement interval in milliseconds CHANGE IF NEEDED
    private static double drift = 0.0;
    

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
    public static void robotCalibrate(){
        gyro.calibrate();
    }

    private static void measureDrift() {
        double startAngle = gyro.getAngle();
        double sumAngle = 0.0;
        long startTime = System.currentTimeMillis();
    
        for (int i = 0; i < DRIFT_MEASUREMENT_SAMPLES; i++) {
            while (System.currentTimeMillis() < startTime + DRIFT_MEASUREMENT_INTERVAL_MS) {
                // wait until the measurement interval has elapsed
            }
            sumAngle += gyro.getAngle() - startAngle;
            startTime += DRIFT_MEASUREMENT_INTERVAL_MS;
        }
    
        double driftRate = sumAngle / DRIFT_MEASUREMENT_INTERVAL_MS / DRIFT_MEASUREMENT_SAMPLES; // the average change in angle per second during each measurement interval
        drift = drift + driftRate * DRIFT_MEASUREMENT_INTERVAL_MS * DRIFT_MEASUREMENT_SAMPLES / 1000.0;// adds the driftrate *duration of each measurement * num samples and converts to seconds
    }
    @Override
    public void periodic() {
        // System.out.println(getGyroAngle());
        // resetGyroAngle();
        measureDrift();
    }

    @Override
    public void simulationPeriodic() {

    }
}
