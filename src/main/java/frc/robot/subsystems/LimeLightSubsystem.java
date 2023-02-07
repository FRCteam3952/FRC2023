package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.NetworkTables;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private final PIDController pidcontrolX, pidcontrolY, pidcontrolAngle;
    private final float kp = 0.003125f;
    private final float ki = 0.01f;
    private final float kd = 0f;

    private float prev_tx = 0f;
    private float prev_ty = 0f;
    private float prev_angle = 0f;

    public LimeLightSubsystem() {
        pidcontrolX = new PIDController(1, ki, kd);
        pidcontrolY = new PIDController(1, ki, kd);
        pidcontrolAngle = new PIDController(1, ki, kd);
    }

    public double getXAdjustment() {
        float tx = (NetworkTables.getLimeLightErrorX() - 160) * kp;

        if(tx > 1){
            tx =1;
        }
        if(tx < -1){
            tx = -1;
        }
        if(tx < 0.01 && tx > -0.01){
            tx = prev_tx;
        }
        else{
            prev_tx = tx;
        }
        double steering_adjust = pidcontrolX.calculate(tx) * -1; // calculate PID
        
        return steering_adjust; 
    }
    public double getYAdjustment() {
        float ty = (NetworkTables.getLimeLightErrorY() - 120) * kp;

        if(ty > 1){
            ty =1;
        }
        if(ty < -1){
            ty = -1;
        }
        if(ty < 0.01 && ty > -0.01){
            ty = prev_ty;
        }
        else{
            prev_ty = ty;
        }
        double steering_adjust = pidcontrolY.calculate(ty) * -1; // calculate PID
        
        return steering_adjust; 
    }
    public double getAngleAdjustment(){
        float angle = (NetworkTables.getConeOrientation()) * kp;

        if(angle > 1){
            angle =1;
        }
        if(angle < -1){
            angle = -1;
        }
        if(angle < 0.01 && angle > -0.01){
            angle = prev_angle;
        }
        else{
            prev_angle = angle;
        }
        double steering_adjust = pidcontrolAngle.calculate(angle) * -1; // calculate PID
        
        return steering_adjust; 
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

}