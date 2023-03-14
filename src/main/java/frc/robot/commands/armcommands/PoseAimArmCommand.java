package frc.robot.commands.armcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.NetworkTablesUtil;

public class PoseAimArmCommand extends CommandBase{
    private final ArmSubsystem arm;
    private final DriveTrainSubsystem drive;
    private double[] coordinates;

    public PoseAimArmCommand(ArmSubsystem arm, DriveTrainSubsystem drive){
        this.arm = arm;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        int targetPoseID = NetworkTablesUtil.getKeyInteger();
        switch(targetPoseID){
            case 1:
                coordinates = PositionConstants.BOTTOM_LEFT_POS;
                break;
            case 2:
                coordinates = PositionConstants.BOTTOM_MIDDLE_POS;
                break;
            case 3:
                coordinates = PositionConstants.BOTTOM_RIGHT_POS;
                break;
            case 4:
                coordinates = PositionConstants.CENTER_LEFT_POS;
                break;
            case 5:
                coordinates = PositionConstants.CENTER_MIDDLE_POS;
                break;
            case 6:
                coordinates = PositionConstants.CENTER_RIGHT_POS;
                break;
            case 7:
                coordinates = PositionConstants.TOP_LEFT_POS;
                break;
            case 8:
                coordinates = PositionConstants.TOP_CENTER_POS;
                break;
            case 9:
                coordinates = PositionConstants.TOP_RIGHT_POS;
                break;
            default:
                System.out.println("A key within 1-9 was not pressed");
                break;
        }

    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPoseInches();
        double targetX = coordinates[0] - currentPose.getX();
        double targetZ = coordinates[2] - currentPose.getY();
        double targetY = coordinates[1];
        arm.setTargetCoordinates(targetX, targetY, targetZ);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // Allowed error subject to change
        return arm.isAtCoords();
    }
}
