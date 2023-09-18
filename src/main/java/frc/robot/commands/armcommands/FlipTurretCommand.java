package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;

public class FlipTurretCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final XboxController xboxController;


    private double speed1;
    private double speed2;

    private double[] coords;

    public FlipTurretCommand(ArmSubsystem arm, XboxController xboxController, double speed1, double speed2) {
        this.arm = arm;
        this.xboxController = xboxController;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coords = ArmConstants.STARTING_COORDS;
        arm.setManualControlMode(false);
        arm.setis2D(false);
        arm.setMaxAndMinOutput1(speed1);
        arm.setMaxAndMinOutput2(speed2);
        arm.setTargetCoordinates(coords[0] * (arm.isAtHumanPlayer() ? -1 : 1), coords[1], 0);
    }

    @Override
    public void execute() {
        System.out.println("flipping turret");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setMaxAndMinOutput1(ArmConstants.MAX_OUTPUT);
        arm.setMaxAndMinOutput2(ArmConstants.MAX_OUTPUT);
        arm.setManualControlMode(true);
        arm.setis2D(true);
        arm.setTargetCoordinates(coords[0], coords[1], 0);
        arm.setTurretSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Allowed error subject to change
        return arm.isAtCoords3D() || Math.abs(xboxController.getLeftHorizontalMovement()) > 0.1 || Math.abs(xboxController.getLeftVerticalMovement()) > 0.1 ||
                Math.abs(xboxController.getRightHorizontalMovement()) > 0.1 || Math.abs(xboxController.getRightVerticalMovement()) > 0.1;
    }


}
