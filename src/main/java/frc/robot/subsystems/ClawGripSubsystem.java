package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawGripSubsystem extends SubsystemBase {
    private final DoubleSolenoid doubleSolenoid;
    private final Compressor compressor;
    private boolean clawClosed;

    public ClawGripSubsystem() {
        this.doubleSolenoid = new DoubleSolenoid(12, PneumaticsModuleType.REVPH, 1, 0);
        this.clawClosed = false;
        this.compressor = new Compressor(12, PneumaticsModuleType.REVPH);
        this.compressor.enableDigital();

    }

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
    public void setClawOpened(boolean state) {
        this.clawClosed = state;
    }

    @Override
    public void periodic() {
        this.doubleSolenoid.set(this.clawClosed ? Value.kForward : Value.kReverse); //open and close claw based on clawClosed boolean
    }

    @Override
    public void simulationPeriodic() {

    }
}
