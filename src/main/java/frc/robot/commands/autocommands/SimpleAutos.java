package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.MathUtil;

/**
 * "Simple" but i mean if it works better than {@link Autos} then i don't care
 * <p>
 * Anyways, we are operating under the assumption that a Trajectory that moves 1 meter will work perfectly. If it doesn't we have bigger issues, so just assume that it works (it worked a few days ago)
 */
public class SimpleAutos {
    /**
     * Is moving away from the drivers (positive) or negative?
     */
    public static final boolean POSITIVE_IS_FORWARD = true; // pls work

    private static final double PICKUP_CONE_POS = 200; // inches

    /**
     * Moves the robot a certain number of meters straight away from the drivers. The robot should be facing parallel to the X axis on the April Tags coordinate system when this is called.
     *
     * @param meters The number of meters to move
     * @param robot  The robot container
     * @return A Command that moves the robot n meters
     */
    public static Command moveNMeters(double meters, RobotContainer robot) {
        Pose2d currentPose = new Pose2d();
        Pose2d newPose = new Pose2d(currentPose.getTranslation().getX() + (POSITIVE_IS_FORWARD ? meters : -meters), currentPose.getTranslation().getY(), currentPose.getRotation());

        return robot.driveTrain.generateRamseteCommand(currentPose, newPose, false);
    }

    /**
     * A test command that moves the robot 1 meter
     *
     * @param robot The robot container
     * @return A Command that moves the robot 1 meter
     */
    public static Command moveOneMeter(RobotContainer robot) {
        return moveNMeters(1, robot);
    }

    public static Command goToEstimatedConeLocation(RobotContainer robot) {
        return moveNMeters(MathUtil.inchesToMeters(PICKUP_CONE_POS), robot);
    }

    public static Command returnFromEstimatedConeLocationToStart(RobotContainer robot) {
        return moveNMeters(MathUtil.inchesToMeters(-PICKUP_CONE_POS), robot);
    }

    /**
     * Copied almost verbatim (changed our homebrew wait command to the official one) from {@link Autos#placeGamePieceAuto(RobotContainer)}
     *
     * @param robot The robot container
     * @return A Command that places a game piece at the top center position.
     */
    public static Command placeGamePieceTopCenter(RobotContainer robot) {
        return Commands.runOnce(
                        () -> { // Makes sure that the claw is closed around game piece
                            System.out.println("Place Cube Auto Start");
                            robot.clawGrip.setClawOpened(false); // Closes claw
                        },
                        robot.clawGrip
                )
                .andThen(robot.goToTopCenter.get()) // Moves arm to top center position on grid to place game piece
                .andThen(new WaitCommand(0.2)) // Waits 0.2 seconds
                .andThen(
                        Commands.runOnce(
                                () -> { // Opens claw to drop game piece onto top center platform
                                    System.out.println("Place Cube Auto Running");
                                    robot.clawGrip.setClawOpened(true); // Opens claw
                                },
                                robot.clawGrip
                        )
                )
                .andThen(new WaitCommand(0.2)) // Waits 0.2 seconds
                .andThen(robot.goToStartingPos.get()); // Moves arm to starting position
    }

    /**
     * Copied almost verbatim (changed our homebrew wait command to the official one) from {@link Autos#doublePlacementAuto(RobotContainer)} (see line 303), but assumes that the cone is already grabbed (which it should be).
     * @param robot The robot container
     * @return A Command that places a game piece at the center right position.
     */
    public static Command placeGamePieceCenterRight(RobotContainer robot) {
        return robot.goToCenterRight.get() // Moves arm to top center position on grid to place game piece
                .andThen(new WaitCommand(0.2)) // Waits 0.2 seconds
                .andThen(
                        Commands.runOnce(
                                () -> { // Opens claw to drop game piece onto top center platform
                                    System.out.println("Place Cube Auto Running");
                                    robot.clawGrip.setClawOpened(true); // Opens claw
                                },
                                robot.clawGrip
                        )
                )
                .andThen(new WaitCommand(0.2)) // Waits 0.2 seconds
                .andThen(robot.goToStartingPos.get()); // Moves arm to starting position
    }

    public static Command doublePlacementAuto(RobotContainer robot) {
        return placeGamePieceTopCenter(robot) // Place the preload at top center
                .andThen(new WaitCommand(0.2)) // wait
                .andThen(
                        robot.goToPickupPosX30.get() // Move the arm to the pickup position while driving the robot, this can be changed to an "andThen" if we want to move the arm first (or swap if drive 1st then arm 2nd)
                                .alongWith(
                                        goToEstimatedConeLocation(robot)
                                )
                )
                .andThen(
                        new WaitCommand(0.5) // wait
                )
                .andThen(
                        Commands.runOnce( // close the claw around the cone (hopefully at least)
                                () -> robot.clawGrip.setClawOpened(false),
                                robot.clawGrip
                        )
                )
                .andThen(
                        new WaitCommand(0.3) // wait
                )
                .andThen(
                        robot.goToStartingPos.get() // bring the arm back to starting BEFORE moving the robot
                )
                .andThen(
                        new WaitCommand(0.2) // wait
                )
                .andThen(
                        returnFromEstimatedConeLocationToStart(robot) // bring the robot back to what is hopefully the starting position
                )
                .andThen(
                        new WaitCommand(0.2) // wait
                )
                .andThen(
                        placeGamePieceCenterRight(robot) // place the cone at center right
                );
    }
}
