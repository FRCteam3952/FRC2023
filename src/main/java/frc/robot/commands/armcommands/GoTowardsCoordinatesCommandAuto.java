package frc.robot.commands.armcommands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class GoTowardsCoordinatesCommandAuto extends CommandBase {

    private final ArmSubsystem arm;

    private final double[] newArmPosition;
    private final double speed1;
    private final double speed2;
    private boolean is2D = true;

    public GoTowardsCoordinatesCommandAuto(ArmSubsystem arm, double[] newArmPosition, double speed1, double speed2) {
        this.arm = arm;
        this.newArmPosition = newArmPosition;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }
    public GoTowardsCoordinatesCommandAuto(ArmSubsystem arm, double[] newArmPosition, double speed1, double speed2, boolean is2D) {
        this.arm = arm;
        this.is2D = is2D;
        this.newArmPosition = newArmPosition;
        this.speed1 = speed1;
        this.speed2 = speed2;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setis2D(this.is2D);
        arm.setMaxAndMinOutput1(speed1);
        arm.setMaxAndMinOutput2(speed2);
        if(this.is2D){
            arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], 0);
        }
        else{
            arm.setTargetCoordinates(newArmPosition[0], newArmPosition[1], newArmPosition[2]);
        }
    }

    @Override
    public void execute() {
        arm.goTowardTargetCoordinates();
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
        if(this.is2D) {
            return arm.isAtCoords();
        }
        else{
            return arm.isAtCoords3D();
        }
    }


}
