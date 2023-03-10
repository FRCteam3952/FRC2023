// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.armcommands.GoTowardsCoordinatesCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;

public final class Autos {
    private static boolean blueTeam = true; // Whether we are on the blue team or not
    private static final Timer timer = new Timer();

    /**
     * Example static factory for an autonomous command.
     */

     // This is used for testing for now
    public static CommandBase exampleAuto(/* if you need a subsystem, pass it in as a parameter here, and make sure to pass them in when the method is called inside RobotContainer */) {
        // You can also pass in commands as parameters if you want to use existing commands as part of your autonomouse command (such as BalanceChargeStation.java *wink wink hint hint*)
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario code
            int i = 0;
            while (i < 100) {
                System.out.println("Auton running1");
                i++;
            }
        }).alongWith(Commands.runOnce(() -> {
            // Autonomous scenario code
            int i = 100;
            while (i > 0) {
                System.out.println("Auton running2");
                i--;
            }
        })).andThen(Commands.runOnce(() -> {
            // Autonomous scenario code
            int i = 100;
            while (i > 0) {
                System.out.println("Auton running3");
                i--;
            }
        })) /* susbsystems that are used inside the curly braces above must be passed in here */;

        // To implement a sequence of actions/commands use .andThen(), can be used to implement PathWeaver trajectories
        // To implement simultaneous actions/commands use .alongWith(), can also be used to implement PathWeaver trajectories
        // For example: return Commands.runOnce(() -> {}).andThen(trajectory1Command);
        // Example of simultaneous implementation: return Commands.runOnce(() -> {}).alongWith(trajectory1Command);
    }

    public static CommandBase defaultAuto() {
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario code
        });

    }

    public static CommandBase balanceAutoFirstHalf(Command driveForwardOverChargeStationBlueCommand, 
            Command driveForwardOverChargeStationRedCommand, ArmSubsystem arm) {

        if (blueTeam) {
            return Commands.runOnce(() -> {
                    // Any neccessary calibration code
                    System.out.println("Balance Auto");
                })/*.alongWith(arm.calibrateArm())*/
                .alongWith(driveForwardOverChargeStationBlueCommand);
        } else {
            return Commands.runOnce(() -> {
                System.out.println("Balance Auto");
                // Any neccessary calibration code
            })/*.alongWith(arm.calibrateArm())*/
            .alongWith(driveForwardOverChargeStationRedCommand);
        }
    }

    public static CommandBase balanceAutoSecondHalf(Command driveBackwardsOntoChargeStationBlueCommand, 
            Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation) {

        if (blueTeam) {    
            return driveBackwardsOntoChargeStationBlueCommand
                .andThen(balanceChargeStation);
        } else {
            return driveBackwardsOntoChargeStationRedCommand
                .andThen(balanceChargeStation);
        }
    }

    // Autonomous mode for balancing charge station
    public static CommandBase balanceAuto(Command driveForwardOverChargeStationBlueCommand, 
            Command driveBackwardsOntoChargeStationBlueCommand, Command driveForwardOverChargeStationRedCommand, 
            Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation, ArmSubsystem arm) {
        
        System.out.println("Balance Auto init");

        return balanceAutoFirstHalf(driveForwardOverChargeStationBlueCommand, driveForwardOverChargeStationRedCommand, arm)
        .andThen(balanceAutoSecondHalf(driveBackwardsOntoChargeStationBlueCommand, driveBackwardsOntoChargeStationRedCommand, balanceChargeStation));
    }

    // Assumes robot is at a AprilTag
    // Might need to add calibration
    public static CommandBase placeConeAuto(ClawGripSubsystem claw, GoTowardsCoordinatesCommand goTowardsTopRight, GoTowardsCoordinatesCommand goTowardsStartingPos) {
        return Commands.runOnce(() -> {
            // Any neccessary calibration code
        }).andThen(goTowardsTopRight)
        .andThen(Commands.runOnce(() -> {
            claw.setClawClosed(false); // open claw
        }, claw).andThen(goTowardsStartingPos));
    }

    // Position values on trajectories may need to be adjusted
    // Adjustments can be made later lol
    // Might need to add calibration 
    public static CommandBase doublePlacementAuto(ArmSubsystem arm, ClawGripSubsystem claw, Command driveBackwardsToCubeBlue, Command driveForwardsToGridBlue, 
            Command driveBackwardsToCubeRed, Command driveForwardsToGridRed, GoTowardsCoordinatesCommand goTowardsTopRight, GoTowardsCoordinatesCommand goTowardsStartingPos,
            GoTowardsCoordinatesCommand goTowardsStartingPos2, GoTowardsCoordinatesCommand goTowardsStartingPos3, GoTowardsCoordinatesCommand goTowardsPickupPos, 
            GoTowardsCoordinatesCommand goTowardsTopCenter) {

        if (blueTeam) {
            return placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos) // Drops pre-loaded cone onto top right pole
            .andThen(driveBackwardsToCubeBlue) // Drives backwards to cube
            .andThen(Commands.runOnce(() -> { 
                claw.setClawClosed(false); // Opens claw
            }).andThen(goTowardsPickupPos) // Arm goes to pickup position
            .andThen(Commands.runOnce(() -> { 
                claw.setClawClosed(true); // Closes claw
            })
            .andThen(goTowardsStartingPos2) // Arm goes to starting position
            .alongWith(driveForwardsToGridBlue) // Drive forwards to grid
            .andThen(goTowardsTopCenter) // Arm goes to top center position on grid
            .andThen(Commands.runOnce(() -> {
                claw.setClawClosed(false); // Open claw
            }).andThen(goTowardsStartingPos3)))); // Arm goes to starting position
        } else {
            return placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos) // Drops pre-loaded cone onto top right pole
            .andThen(driveBackwardsToCubeRed) // Drives backwards to cube
            .andThen(Commands.runOnce(() -> { 
                claw.setClawClosed(false); // Opens claw
            }).andThen(goTowardsPickupPos) // Arm goes to pickup position
            .andThen(Commands.runOnce(() -> { 
                claw.setClawClosed(true); // Closes claw
            })
            .andThen(goTowardsStartingPos2) // Arm goes to starting position
            .alongWith(driveForwardsToGridRed) // Drive forwards to grid
            .andThen(goTowardsTopCenter) // Arm goes to top center position on grid
            .andThen(Commands.runOnce(() -> {
                claw.setClawClosed(false); // Open claw
            }).andThen(goTowardsStartingPos3)))); // Arm goes to starting position
        }
    }

    // Might need to add calibration
    public static CommandBase placeConeThenBalanceAuto(Command driveForwardOverChargeStationBlueCommand, Command driveBackwardsOntoChargeStationBlueCommand, 
            Command driveForwardOverChargeStationRedCommand, Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation, ArmSubsystem arm, ClawGripSubsystem claw,
            GoTowardsCoordinatesCommand goTowardsTopRight, GoTowardsCoordinatesCommand goTowardsStartingPos) {

        return balanceAutoFirstHalf(driveForwardOverChargeStationBlueCommand, driveForwardOverChargeStationRedCommand, arm)
        .andThen(placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos))
        .andThen(balanceAutoSecondHalf(driveBackwardsOntoChargeStationBlueCommand, driveBackwardsOntoChargeStationRedCommand, balanceChargeStation));
        
    }

    private Autos() {
        throw new UnsupportedOperationException("Autos is a utility class and cannot be instantiated!");
    }
}
