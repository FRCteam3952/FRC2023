package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;

/**
 * This is here for implementation in Autonomous mode, for teleop there is already an implementation in ArmControlCommand
 */
public class PickupPieceCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final ClawGripSubsystem claw;
    private final XboxController controller;

    private final double X_SPEED = 0.8;
    //private final double Y_SPEED = 0.8;
    private final double TURRET_SPEED = 1;
    private Timer timer = new Timer();

    private enum states {
        AIM,
        CLOSE_CLAW,
        RETRACT,
        END
    }

    private states currentState;
    private double height;

    public PickupPieceCommand(ArmSubsystem arm, ClawGripSubsystem claw, XboxController controller, double height) {
        this.arm = arm;
        this.claw = claw;
        this.controller = controller;
        this.height = height;
        addRequirements(arm);
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.currentState = states.AIM;
        timer.start();
        double[] currentCoords = arm.getTargetCoordinates();
        this.arm.setTargetCoordinates(currentCoords[0], this.height, currentCoords[2]); //set the wanted height of the arm
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (currentState) { //use limelight and ultrasonic sensor to move to cone or cube
            case AIM:
                double[] adjustments = LimeLight.getAdjustmentFromError(this.arm.getFlipped());
                this.arm.moveVector(adjustments[0] * X_SPEED, 0, 0);
                this.arm.setTurretSpeed(TURRET_SPEED * adjustments[2]);

                if (adjustments[0] + adjustments[1] + adjustments[2] < 5) {
                    this.currentState = states.CLOSE_CLAW;
                    timer.reset();
                }
                break;

            case CLOSE_CLAW: //close claw
                claw.setClawOpened(false);
                if (timer.get() > 0.2) {
                    this.currentState = states.RETRACT;
                }
                break;

            case RETRACT: //move arm back to position
                (new CalibrateArmPivotsCommand(this.arm, this.controller)).schedule();
                this.currentState = states.END;
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
