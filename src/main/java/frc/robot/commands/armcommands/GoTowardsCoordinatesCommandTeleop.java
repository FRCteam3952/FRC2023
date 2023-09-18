package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;

public class GoTowardsCoordinatesCommandTeleop extends CommandBase {

    private final ArmSubsystem arm;
    private final XboxController xboxController;


    private double[] newArmPosition;
    private double speed1;
    private double speed2;

    private boolean is2D = false;

    public GoTowardsCoordinatesCommandTeleop(ArmSubsystem arm, double[] newArmPosition, XboxController xboxController, double speed1, double speed2) {
        this.arm = arm;
        this.newArmPosition = newArmPosition;
        this.xboxController = xboxController;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }

    public GoTowardsCoordinatesCommandTeleop(ArmSubsystem arm, double[] newArmPosition, XboxController xboxController, double speed1, double speed2, boolean is2D) {
        this.is2D = is2D;
        this.arm = arm;
        this.newArmPosition = newArmPosition;
        this.xboxController = xboxController;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setis2D(is2D);
        arm.setManualControlMode(false);
        arm.setMaxAndMinOutput1(speed1);
        arm.setMaxAndMinOutput2(speed2);
        if (is2D){
            arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], 0);
        }
        else{
            arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], newArmPosition[2]);
        }

    }

    @Override
    public void execute() {
        System.out.println("RNIGNAUU");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setMaxAndMinOutput1(ArmConstants.MAX_OUTPUT);
        arm.setMaxAndMinOutput2(ArmConstants.MAX_OUTPUT);
        arm.setManualControlMode(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Allowed error subject to change
        return arm.isAtCoords() || Math.abs(xboxController.getLeftHorizontalMovement()) > 0.1 || Math.abs(xboxController.getLeftVerticalMovement()) > 0.1 ||
                Math.abs(xboxController.getRightHorizontalMovement()) > 0.1 || Math.abs(xboxController.getRightVerticalMovement()) > 0.1;
    }


}
