package frc.robot.commands.armcommands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class GoTowardsCoordinatesCommandAuto extends CommandBase {

    private final ArmSubsystem arm;

    private double[] newArmPosition;
    private double speed1;
    private double speed2;

    public GoTowardsCoordinatesCommandAuto(ArmSubsystem arm, double[] newArmPosition, double speed1, double speed2) {
        this.arm = arm;
        this.newArmPosition = newArmPosition;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setMaxAndMinOutput1(speed1);
        arm.setMaxAndMinOutput2(speed2);
        arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], newArmPosition[2]);
    }

    @Override
    public void execute() {
        arm.goTowardTargetCoordinates();
        System.out.println("BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS BALLS");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("WE ARE AT COORDINATES: " + Arrays.toString(this.arm.getCurrentAnglesDeg()) + ", going to: " + Arrays.toString(this.arm.getTargetAngles()));
        arm.setMaxAndMinOutput1(ArmConstants.MAX_OUTPUT);
        arm.setMaxAndMinOutput2(ArmConstants.MAX_OUTPUT);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Allowed error subject to change
        return arm.isAtCoords();
    }


}
