package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.NetworkTables;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private static PIDController clawRotationPID;
    private static final float kp = 0.003125f;
    private static final float ki = 0.01f;
    private static final float kd = 0f;

    public LimeLightSubsystem() {
        clawRotationPID = new PIDController(1, ki, kd);
    }

    public static float getXAdjustment() {
        float tx = (NetworkTables.getLimeLightErrorX() - 160) * kp;
        
        // if tx is too big, return the max of 1 or -1
        if (Math.abs(tx) > 1) {
            // return 1 if tx is greater than 1, -1 if tx is less than -1
            return Math.copySign(1, tx);
        }
        return tx;
    }

    public static float getYAdjustment() {
        float ty = (NetworkTables.getLimeLightErrorY() - 120) * kp;

        // if ty is too big, return the max of 1 or -1
        if (Math.abs(ty) > 1) {
            // return 1 if ty is greater than 1, -1 if ty is less than -1
            return Math.copySign(1, ty);
        }
        return ty;
    }

    public static double getAngleAdjustment(){
        float angle = (NetworkTables.getConeOrientation()) * kp;
  
        // if angle is too big, return the max of 1 or -1
        if (Math.abs(angle) > 1) {
            // return 1 if angle is greater than 1, -1 if angle is less than -1
            return Math.copySign(1, angle);
        }
        // calculate the PID for the steering adjustment
        return -clawRotationPID.calculate(angle, angle > 180 ? 360 : 0); // Negated because claw rotation angle is inversely related to cone orientation angle
        // If cone angle measures greater than 180 (tip pointing towards right), it goes towards 360. If it measures less than 180 (tip pointing towards left), it goes towards 0. (360 and 0 both represent the cone pointing straight up)
    }

    public static void setIntendedAngle(double setpoint) {
        clawRotationPID.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

}