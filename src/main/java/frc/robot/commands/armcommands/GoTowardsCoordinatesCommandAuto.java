package frc.robot.commands.armcommands;

import frc.robot.controllers.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class GoTowardsCoordinatesCommandAuto extends CommandBase{

    private final ArmSubsystem arm;

    private double[] newArmPosition;
    private double[] currentArmPosition;
    
    public GoTowardsCoordinatesCommandAuto(ArmSubsystem arm, double[] newArmPosition) {
        this.arm = arm;
        this.newArmPosition = ArmConstants.STARTING_COORDS;
        this.currentArmPosition = arm.getCurrentCoordinates();
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentArmPosition = arm.getCurrentCoordinates();
        arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], newArmPosition[2]);
    }

    @Override
    public void execute() {
        arm.goTowardTargetCoordinates();
        currentArmPosition = arm.getCurrentCoordinates();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Allowed error subject to change
        return arm.isAtCoords();
    }



}
