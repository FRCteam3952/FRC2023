package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class ClawGripSubsystem extends SubsystemBase {
    // private final CANSparkMax clawGrip;
    // private final RelativeEncoder clawGripEncoder;
    private final DoubleSolenoid doubleSolenoid;
    private final Compressor compressor;
    private boolean clawClosed;
    DigitalInput limit = new DigitalInput(PortConstants.CLAW_LIMIT_SWITCH_PORT);

    public ClawGripSubsystem() {
        // this.clawGrip = new CANSparkMax(PortConstants.CLAW_GRIP_PORT, MotorType.kBrushless);
        this.doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0); // Change to the correct things
        // this.clawGripEncoder = this.clawGrip.getEncoder();
        // this.clawGripEncoder.setPosition(0);?":"
        this.clawClosed = false;
        this.compressor = new Compressor(PneumaticsModuleType.REVPH);
        this.compressor.enableDigital();

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
     *
     * @return True for closing, False for not closing
     */
    public boolean getClawClosed() {
        return this.clawClosed; // rename variable to more descriptive?
    }

    /**
     * Sets the claw state
     *
     * @param state True for closing, False for not closing
     */
    public void setClawClosed(boolean state) {
        this.clawClosed = state;
    }

    @Override
    public void periodic() {
        this.doubleSolenoid.set(this.clawClosed ? Value.kForward : Value.kReverse);
        System.out.println("SOLENOID STATE: " + this.doubleSolenoid.get().name() + ", FWD/REV Disabled: " + this.doubleSolenoid.isFwdSolenoidDisabled() + "/" + this.doubleSolenoid.isRevSolenoidDisabled());
        System.out.println("COMPRESSOR STATE- Full:" + this.compressor.getPressureSwitchValue() + ", Enabled: " + this.compressor.isEnabled() + ", Current: " + this.compressor.getCurrent());
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
