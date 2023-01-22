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
        this.clawState = false;
    }

    // Runs once when claw grip trigger is released
    public CommandBase openClaw() {
        return this.runOnce(
            () -> {
              while (clawGripEncoder.getPosition() < ClawConstants.MAX_GRIP_ENCODER_VALUE) {
                clawGrip.set(ClawConstants.CLAW_GRIP_SPEED); // find out correct direction later
              }
              clawGrip.set(0);
            });
    }

    // Runs continuously when claw grip trigger is held down
    public CommandBase closeClaw() {
        return this.runOnce(
            () -> {
              if (clawGripEncoder.getPosition() > ClawConstants.MIN_GRIP_ENCODER_VALUE) {
                clawGrip.set(-ClawConstants.CLAW_GRIP_SPEED); // find out correct direction later
              } else {
                clawGrip.set(0);
              }
            });
    }

    public boolean getClawState() {
        return this.clawState; // rename variable to more descriptive?
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        
    }
}
