package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.NetworkTables;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private final PIDController pidcontrolAngle;
    private final float kp = 0.003125f;
    private final float ki = 0.01f;
    private final float kd = 0f;

    private float prev_tx = 0f;
    private float prev_ty = 0f;
    private float prev_angle = 0f;

    public LimeLightSubsystem() {
        pidcontrolAngle = new PIDController(1, ki, kd);
    }

    public float getXAdjustment() {
        float tx = (NetworkTables.getLimeLightErrorX() - 160) * kp;
        
        // if tx is too big, return the max of 1 or -1
        if (Math.abs(tx) > 1) {
            // return 1 if tx is greater than 1, -1 if tx is less than -1
            return Math.copySign(1, tx);
        }
        // if the value is real close to 0, return the previous value instead
        if(Math.abs(tx) < 0.01){
            return prev_tx;
        }
        prev_tx = tx;
        return tx;
    }

    public float getYAdjustment() {
        float ty = (NetworkTables.getLimeLightErrorY() - 120) * kp;

        // if ty is too big, return the max of 1 or -1
        if (Math.abs(ty) > 1) {
            // return 1 if ty is greater than 1, -1 if ty is less than -1
            return Math.copySign(1, ty);
        }
        // if the value is very close to 0, return the previous value instead
        if(Math.abs(ty) < 0.01){
            return prev_ty;
        }
        prev_ty = ty;
        return ty;
    }

    public double getAngleAdjustment(){
        float angle = (NetworkTables.getConeOrientation()) * kp;

        // if angle is too big, return the max of 1 or -1
        if (Math.abs(angle) > 1) {
            // return 1 if angle is greater than 1, -1 if angle is less than -1
            return Math.copySign(1, angle);
        }
        // if the value is very close to 0, return the previous value instead
        if(Math.abs(angle) < 0.01) {
            angle = prev_angle;
        }
        else {
            prev_angle = angle;
        }
        // calculate the PID for the steering adjustment
        return -pidcontrolAngle.calculate(angle);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

}