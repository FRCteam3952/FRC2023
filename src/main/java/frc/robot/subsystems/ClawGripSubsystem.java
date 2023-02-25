package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;

public class ClawGripSubsystem extends SubsystemBase {
    // private final CANSparkMax clawGrip;
    // private final RelativeEncoder clawGripEncoder;
    private final DoubleSolenoid doubleSolenoid;
    private boolean clawClosed;
    DigitalInput limit = new DigitalInput(PortConstants.CLAW_LIMIT_SWITCH_PORT);

    public ClawGripSubsystem() {
        // this.clawGrip = new CANSparkMax(PortConstants.CLAW_GRIP_PORT, MotorType.kBrushless);
        this.doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // Change to the correct things
        doubleSolenoid.set(Value.kOff);
        // this.clawGripEncoder = this.clawGrip.getEncoder();
        // this.clawGripEncoder.setPosition(0);?":"
        this.clawClosed = false;
    }

    // public void setSpeed(double speed) {
    //     if (speed > 0 && limit.get()) {
    //         this.clawGrip.set(0);
    //     } else {
    //         this.clawGrip.set(speed);
    //     }
    // }

    public void setForward() {
        this.doubleSolenoid.set(Value.kForward);
    }

    public void setReverse() {
        this.doubleSolenoid.set(Value.kReverse);
    }

    public void setOff() {
        this.doubleSolenoid.set(Value.kOff);
    }
    

    /**
     * Gets the claw state
     * @return True for closing, False for not closing
     */
    public boolean getClawClosed() {
        return this.clawClosed; // rename variable to more descriptive?
    }

    /**
     * Sets the claw state
     * @param state True for closing, False for not closing
     */
    public void setClawClosed(boolean state) {
        this.clawClosed = state;
    }

    @Override
    public void periodic() {
        // if(this.limit.get() && this.clawGrip.get() > 0) {
        //     this.clawGrip.set(0);
        // }

        // System.out.println("CLAW SPEED " + this.clawGrip.get() + ", CLAW CLOSED STATE: " + this.getClawClosed());
        // System.out.println("CLAW ENCODER: " + this.clawGripEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {

    }
}
