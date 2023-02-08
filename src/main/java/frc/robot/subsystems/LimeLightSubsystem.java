package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.NetworkTables;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private static PIDController clawRotationPID;

    private static final float kp = 0.003125f;
    private static final float ki = 0.01f;
    private static final float kd = 0f;

    // These are for referring to specific list values in 'previous' or 'modifiers'.
    // The first element in 'previous' is the previous value from NetworkTables for the TX adjustment, 
    // the second is for TY, and the third is for ANGLE. 

    private static final int TX = 0;
    private static final int TY = 1;
    private static final int ANGLE = 2;

    // referred to by TX, TY, or ANGLE. Each of these will be updated.
    private static float[] previous = new float[]{
        0f, // TX int refers to this value, which will change from this starting value.
        0f, // TY is 1 and will refer to this value
        0f  // this is the previous value for ANGLE
    };

    // I don't know why these modifiers are needed, but they were in the original code.
    // Someone who knows what these do needs to give this array a better name.
    private static int[] modifiers  = new int[]{
        -160, // this modifies the return value of getLimeLightError for TX
        -120, // this is for for TY
        0     // this is for ANGLE
    };

    public LimeLightSubsystem() {
        clawRotationPID = new PIDController(1, ki, kd);
    }

    private static float getInitialValueFor(int adjustmentType) {
        // gets the right value from the networks tables based on the type of adjustment
        // the type of adjustment the value is for is either TX, TY, or ANGLE.
        if (adjustmentType == TX) {
            // TX means we need the errorX
            return NetworkTables.getLimeLightErrorX();
        } else if (adjustmentType == TY) {
            // TY means we need the errorY
            return NetworkTables.getLimeLightErrorY();
        } else {
            // ANGLE means we need the cone's orientation
            return NetworkTables.getConeOrientation();
        }
    }

    // adjustmentType is TX, TY, or ANGLE
    public static float getAdjustment(int adjustmentType) {
        // I don't know what the modifiers or 'kp' do, someone else needs to comment about that.
        float initialValue = (getInitialValueFor(adjustmentType) - modifiers[adjustmentType]) * kp;

        // if the value is too big, return the maximums of 1 or -1
        if (Math.abs(initialValue) > 1) {
            // returns the sign of the number, 1 or -1
            return Math.signum(initialValue);
        }
        // if the value is too small, return the previous value instead.
        else if (Math.abs(initialValue) < 0.01) {
            return previous[adjustmentType];
        } 
        // otherwise everything is fine, we update the previous value and return the value we got from the network tables. 
        else {
            previous[adjustmentType] = initialValue;
            return initialValue;
        }
    }

    public static float getXAdjustment() {
        // just use the custom method above
        return getAdjustment(TX);
    }

    public static float getYAdjustment() {
        // just use the custom method above
        return getAdjustment(TY);
    }

    public static double getAngleAdjustment(){
        float angle = getAdjustment(ANGLE);
        // extra steps for calculations

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