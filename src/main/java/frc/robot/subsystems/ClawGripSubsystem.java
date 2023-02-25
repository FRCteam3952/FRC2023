package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ControllerConstants;

public class ClawGripSubsystem extends SubsystemBase {
    private final CANSparkMax clawGrip;
    private final RelativeEncoder clawGripEncoder;
    private boolean clawState;
    private final DigitalInput claw1Limit;
    public ClawGripSubsystem() {
        this.clawGrip = new CANSparkMax(PortConstants.CLAW_GRIP_PORT, MotorType.kBrushless);
        this.clawGripEncoder = this.clawGrip.getEncoder();
        this.clawState = false;
        this.claw1Limit = new DigitalInput(ControllerConstants.CLAW_GRIP_BUTTON_NUMBER);//Change Constant Value

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
    public boolean getClawLimitPressed() {
        return !this.claw1Limit.get();
    }
   

    public boolean getClawState() {
        return this.clawState; // rename variable to more descriptive?
    }

    @Override
    public void periodic() {
        if(getClawLimitPressed()){
            clawGrip.set(0.0);
            clawGripEncoder.setPosition(ClawConstants.MIN_GRIP_ENCODER_VALUE);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
