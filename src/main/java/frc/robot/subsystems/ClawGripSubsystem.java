package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ClawConstants;

public class ClawGripSubsystem extends SubsystemBase {
    private final CANSparkMax clawGrip;
    private final RelativeEncoder clawGripEncoder;
    private boolean clawState;

    public ClawGripSubsystem() {
        this.clawGrip = new CANSparkMax(PortConstants.CLAW_GRIP_PORT, MotorType.kBrushless);
        this.clawGripEncoder = this.clawGrip.getEncoder();
        this.clawGripEncoder.setPosition(0);
        this.clawState = false;
    }

    // Runs once when claw grip trigger is released
    public CommandBase openClaw() {
        return this.runOnce(
                () -> {
                    if (-clawGripEncoder.getPosition() < ClawConstants.MAX_GRIP_ENCODER_VALUE) {
                        System.out.println("OPENED CLAW");
                        clawGrip.set(ClawConstants.CLAW_GRIP_SPEED); // find out correct direction later
                    } else {
                        System.out.println("STOPPED CLAW");
                        clawGrip.set(0);
                    }
                });
    }

    // Runs continuously when claw grip trigger is held down
    public CommandBase closeClaw() {
        return this.runOnce(
                () -> {
                    if (-clawGripEncoder.getPosition() > ClawConstants.MIN_GRIP_ENCODER_VALUE) {
                        System.out.println("CLOSED CLAW");
                        clawGrip.set(-ClawConstants.CLAW_GRIP_SPEED); // find out correct direction later
                    } else {
                        System.out.println("STOPPED CLAW");
                        clawGrip.set(0);
                    }
                });
    }

    public CommandBase stopClaw() {
        return this.runOnce(
            () -> {
                System.out.println("STOPPED CLAW");
                this.clawGrip.set(0);
            }
        );
    }

    public boolean getClawState() {
        return this.clawState; // rename variable to more descriptive?
    }

    @Override
    public void periodic() {
        // System.out.println("CLAW ENCODER: " + this.clawGripEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {

    }
}
