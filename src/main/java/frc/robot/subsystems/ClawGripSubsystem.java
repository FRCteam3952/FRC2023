package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;

public class ClawGripSubsystem extends SubsystemBase {
    private final CANSparkMax clawGrip;
    private final RelativeEncoder clawGripEncoder;
    private boolean clawState;

    public ClawGripSubsystem() {
        this.clawGrip = new CANSparkMax(PortConstants.CLAW_GRIP_PORT, MotorType.kBrushless);
        this.clawGripEncoder = this.clawGrip.getEncoder();
        this.clawState = false;
    }

    public void openClaw(){
        // TODO: IMPLEMENT
    }

    public void closeClaw(){
        // TODO: IMPLEMENT
    }

    public boolean getClawState(){
        return this.clawState; // rename variable to more descriptive?
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        
    }
}
