package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * This is here for implementation in Autonomous mode, for teleop there is already an implementation in ArmControlCommand
 */
public class PickupConeCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final ClawGripSubsystem claw;
    private static final double X_SPEED = 0.8;
    private static final double Y_SPEED = 0.8;
    private static final double TURRET_SPEED = 1;
    private static Timer timer = new Timer();
    private static enum states {
        AIM,
        CLOSE_CLAW,
        RETRACT,
        END
    }
    private static states currentState;
    
    public PickupConeCommand(ArmSubsystem arm, ClawGripSubsystem claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm);
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentState = states.AIM;
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch(currentState){
            case AIM:
                double[] adjustments = LimeLight.getAdjustmentFromError(this.arm.getFlipped());
                arm.moveVector(adjustments[0] * X_SPEED, adjustments[1] * Y_SPEED, 0);
                this.arm.setTurretSpeed(TURRET_SPEED * adjustments[2]); 

                if(adjustments[0] + adjustments[1] + adjustments[2] < 5){
                    currentState = states.CLOSE_CLAW;
                    timer.reset();
                }
                break;

            case CLOSE_CLAW:
                claw.setClawOpened(false);
                if (timer.get() > 0.2){
                    currentState = states.RETRACT;
                }
                break;

            case RETRACT:
                arm.calibrateArm();
                currentState = states.END;
                break;

            default:
                break;
        }
  

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentState == states.END;
    }
}
