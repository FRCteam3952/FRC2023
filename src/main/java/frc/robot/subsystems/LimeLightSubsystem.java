package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final PIDController pidcontrol;
    private final float kp = 0.003125f;
    private final float ki = 0.01f;
    private final float kd = 0f;

    private float prev_tx = 0f;

    public LimeLightSubsystem() {
        pidcontrol = new PIDController(1, ki, kd);
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("limelight"); // initiate limelight table
    }

    public double getAdjustment() {
        float info[] = table.getEntry("llpython").getFloatArray(new float[3]);
        float tx = (info[1] - 160) * kp;

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
        double steering_adjust = pidcontrol.calculate(tx) * -1; // calculate PID
        
        return steering_adjust;
    }

    public void turnOnLED() {
        table.getEntry("ledMode").setDouble(3);
    }

    
    public void turnOffLED() {
        table.getEntry("ledMode").setDouble(1);
    }

    @Override
    public void periodic() {
        //System.out.println(table.getEntry("ledMode").getDouble(-69));
    }

    @Override
    public void simulationPeriodic() {

    }

}