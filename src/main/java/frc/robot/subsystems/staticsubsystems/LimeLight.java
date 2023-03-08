package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.NetworkTablesUtil;

public class LimeLight {
    private static final double DESIRED_AREA_CONE = 5000; // tentative measurement, pixels
    private static final double DESIRED_AREA_CUBE = 420; // measure later

    private static final PIDController clawRotationPID;
    private static final float kp = 0.003125f;
    private static final float ki = 0.01f;
    private static final float kd = 0f;

    static {
        clawRotationPID = new PIDController(1, ki, kd);
    }

    public static void poke() {
        System.out.println("LimeLight initialized");
    }

    public LimeLight() {
    }

    public static float getXAdjustment() {
        float tx = (NetworkTablesUtil.getLimeLightErrorX() - 160) * kp;

        // if tx is too big, return the max of 1 or -1
        if (Math.abs(tx) > 1) {
            // return 1 if tx is greater than 1, -1 if tx is less than -1
            return Math.copySign(1, tx);
        }
        return tx;
    }

    public static float getYAdjustment() {
        float ty = (NetworkTablesUtil.getLimeLightErrorY() - 120) * kp;

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

    public static double getAngleAdjustment() {
        float angle = (NetworkTablesUtil.getConeOrientation()) * kp;

        // if angle is too big, return the max of 1 or -1
        if (Math.abs(angle) > 1) {
            // return 1 if angle is greater than 1, -1 if angle is less than -1
            return Math.copySign(1, angle);
        }
        // calculate the PID for the steering adjustment
        return -clawRotationPID.calculate(angle, angle > 180 ? 360 : 0); // Negated because claw rotation angle is inversely related to cone orientation angle
        // If cone angle measures greater than 180 (tip pointing towards right), it goes towards 360. If it measures less than 180 (tip pointing towards left), it goes towards 0. (360 and 0 both represent the cone pointing straight up)
    }

    // Gets adjustments from limelight and converts them to position adjustments
    public static double[] getAdjustmentFromError(boolean flipped){
        double[] adjustments = new double[3];

        if(flipped){

            double yAdjustment = NetworkTablesUtil.getLimeLightPipeline() == 1 ? (DESIRED_AREA_CONE - getArea()) / DESIRED_AREA_CONE : 
                    (DESIRED_AREA_CUBE - getArea()) / DESIRED_AREA_CUBE; // y axis from perspective of the camera
            yAdjustment = yAdjustment > 1 ? 1 : yAdjustment;
    
            adjustments[0] = getYAdjustment(); // x-axis adjustment
    
            adjustments[1] = yAdjustment; // y-axis adjustment
    
            adjustments[2] = getXAdjustment(); // z-axis adjustment
    
        }
        else{

            double xAdjustment = NetworkTablesUtil.getLimeLightPipeline() == 1 ? (DESIRED_AREA_CONE - getArea()) / DESIRED_AREA_CONE : 
                    (DESIRED_AREA_CUBE - getArea()) / DESIRED_AREA_CUBE; // z axis from perspective of the camera
            xAdjustment = xAdjustment > 1 ? 1 : xAdjustment;
    
            adjustments[0] = xAdjustment; // x-axis adjustment
    
            adjustments[1] = getYAdjustment(); // y-axis adjustment
    
            adjustments[2] = getXAdjustment(); // z-axis adjustment
    
        }
            
        return adjustments;

    }

    public static void setIntendedAngle(double setpoint) {
        clawRotationPID.setSetpoint(setpoint);
    }
}