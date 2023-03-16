package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.util.MathUtil;

public class UltrasonicSensor {
    private static AnalogInput distanceFinder;
    private static final int SCALING_FACTOR = 1;

    static {
        distanceFinder = new AnalogInput(0); //port 0
        distanceFinder.setAverageBits(2); // change later
    }

    public static double getDistanceInches() {
        double volts = distanceFinder.getAverageVoltage();
        return (double) (volts * SCALING_FACTOR);
    }

    public static double getDistanceMeters() {
        return MathUtil.inchesToMeters(getDistanceInches());
    }
}
