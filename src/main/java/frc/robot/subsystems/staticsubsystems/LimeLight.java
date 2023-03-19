package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.NetworkTablesUtil;

public class LimeLight {
    private static final double DESIRED_AREA_CONE = 5000; // tentative measurement, pixels
    private static final double DESIRED_AREA_CUBE = 420; // measure later
    private static final double kp = 0.00169;
    private static final double ki = 0.00;
    private static final double kd = 0.00;
    private static final PIDController adjustmentPID = new PIDController(kp, ki, kd);
    private static final PIDController adjustmentPID2 = new PIDController(kp, ki, kd);

    public static void poke() {
        System.out.println("LimeLight initialized");
    }

    public LimeLight() {
    }


    public static double getXAdjustment() {
        double tx = adjustmentPID.calculate(NetworkTablesUtil.getLimeLightErrorX());
        // if tx is too big, return the max of 1 or -1
        if (Math.abs(tx) > 1) {
            // return 1 if tx is greater than 1, -1 if tx is less than -1
            return Math.copySign(1, tx);
        }
        return tx;
    }
    public static double getDistance(){
        return 0;
    }

    public static double getYAdjustment() {
        double ty = adjustmentPID2.calculate(NetworkTablesUtil.getLimeLightErrorY());

        // if ty is too big, return the max of 1 or -1
        if (Math.abs(ty) > 1) {
            // return 1 if ty is greater than 1, -1 if ty is less than -1
            return Math.copySign(1, ty);
        }
        return ty;
    }

    public static float getArea() {
        float tA = (NetworkTablesUtil.getLimeLightArea());
        return tA;
    }

    public static double getAngle() {
        float angle = (NetworkTablesUtil.getConeOrientation());
        angle = angle > 180 ? angle - 360 : angle;

        // calculate the PID for the steering adjustment
        return angle;
        // If cone angle measures greater than 180 (tip pointing towards right), it goes towards 360. If it measures less than 180 (tip pointing towards left), it goes towards 0. (360 and 0 both represent the cone pointing straight up)
    }

    // Gets adjustments from limelight and converts them to position adjustments
    // Super messy right now, TODO: clean up later
    public static double[] getAdjustmentFromError(boolean flipped) {
        double[] adjustments = new double[3];

        if (flipped) {

            adjustments[0] = 0;

            adjustments[1] = 0; // y-axis adjustment

            adjustments[2] = getXAdjustment(); // z-axis adjustment

        } else {

            double xAdjustment = NetworkTablesUtil.getLimeLightPipeline() == 1 ? (DESIRED_AREA_CONE - getArea()) / DESIRED_AREA_CONE :
                    (DESIRED_AREA_CUBE - getArea()) / DESIRED_AREA_CUBE; // z axis from perspective of the camera
            xAdjustment = xAdjustment > 1 ? 1 : xAdjustment;

            adjustments[0] = getDistance(); // x-axis adjustment

            adjustments[1] = 0;

            adjustments[2] = getXAdjustment(); // z-axis adjustment

        }

        return adjustments;

    }
}