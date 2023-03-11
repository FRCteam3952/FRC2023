// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.commands.armcommands.GoTowardsCoordinatesCommandAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.util.NetworkTablesUtil;

public final class Autos {
    private static boolean blueTeam = NetworkTablesUtil.getIfOnBlueTeam(); // Whether we are on the blue team or not
    private static Timer timer = new Timer();
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
            System.out.println("Default Auto Running");
            // Autonomous scenario code
        });

    }

    // 6.9 seconds driving at 0.25 power goes RIGHT to the edge of the community
    public static CommandBase taxiAuto(DriveTrainSubsystem driveTrain) {
        return Commands.runOnce(() -> {
            System.out.println("Taxi Auto Start");
            timer.reset();
            timer.start();
        }).andThen(Commands.run(() -> {      
            if (timer.get() < 1.15) {
                System.out.println("Slow Drive");
                driveTrain.tankDrive(0.25, 0); // Drives backwards slowly to edge of charge station for 1.15 seconds
            } else if (timer.get() < 2.65) {
                System.out.println("Fast Drive");
                driveTrain.tankDrive(0.5, 0); // Drives faster up onto charge station for 1.5 seconds
            } else {
                driveTrain.tankDrive(0, 0);
                System.out.println("Taxi Auto Finish");          
            }
        }, driveTrain));
    }

    public static CommandBase taxiForBalanceAuto(DriveTrainSubsystem driveTrain) {
        return Commands.runOnce(() -> {
            System.out.println("Taxi For Balance Auto Start");
            timer.reset();
            timer.start();
        }).andThen(Commands.run(() -> {      
            if (timer.get() < 1.15) {
                System.out.println("Slow Drive Backwards");
                driveTrain.tankDrive(0.25, 0); // Drives backwards slowly to edge of charge station for 1.15 seconds
            } else if (timer.get() < 3.65) {
                System.out.println("Fast Drive Backwards");
                driveTrain.tankDrive(0.5, 0); // Drives backwards faster over charge station for 2.5 seconds
            } else if (timer.get() < 5.15) {
                System.out.println("Fast Drive Forwards");
                driveTrain.tankDrive(-0.5, 0); // Drives forwards onto charge station for 1.5 seconds
            } else {
                driveTrain.tankDrive(0, 0);
                System.out.println("Taxi For Balance Auto Finish");          
            }
        }, driveTrain));
    }

    public static CommandBase placeCubeThenTaxiAuto(DriveTrainSubsystem driveTrain, ClawGripSubsystem claw, Command goToTopCenter, Command goToStartingPos) {
        return placeCubeAuto(claw, goToTopCenter, goToStartingPos)
        .andThen(taxiAuto(driveTrain));
    }

    public static CommandBase placeCubeAuto(ClawGripSubsystem claw, Command goToTopCenter, Command goToStartingPos) {
        return Commands.runOnce(() -> {
            System.out.println("Place Cube Auto Start");
            claw.setClawOpened(false);
        }, claw).andThen(goToTopCenter)
        .andThen(Commands.runOnce(() -> {
            timer.reset();
            timer.start();
        })).andThen(Commands.run(() -> {
            // Wait
            System.out.println("Waiting");
        }).until(() -> timer.get() > 0.5))
        .andThen(Commands.runOnce(() -> {
            System.out.println("Place Cube Auto Running");
            claw.setClawOpened(true); // Opens claw
        }, claw))
        .andThen(Commands.runOnce(() -> {
            timer.reset();
            timer.start();
        })).andThen(Commands.run(() -> {
            // Wait
            System.out.println("Waiting");
        }).until(() -> timer.get() > 0.5))
        .andThen(goToStartingPos);
    }

    public static CommandBase taxiThenBalanceAuto(DriveTrainSubsystem driveTrain, Command balanceCommand) {
        
        return Commands.runOnce(() -> {
            System.out.println("Taxi Auto then Balance Start");
        }).andThen(taxiForBalanceAuto(driveTrain).until(() -> timer.get() > 6))
        .andThen(balanceCommand);
    }

    public static CommandBase placeCubeThenTaxiThenBalanceAuto(DriveTrainSubsystem driveTrain, ClawGripSubsystem claw, Command goToTopCenter, Command goToStartingPos, Command balanceCommand) {
        return placeCubeAuto(claw, goToTopCenter, goToStartingPos)
        .andThen(taxiThenBalanceAuto(driveTrain, balanceCommand));
    }

    // First half of balance auto
    public static CommandBase balanceAutoFirstHalf(Command driveForwardOverChargeStationBlueCommand, 
            Command driveForwardOverChargeStationRedCommand, ArmSubsystem arm) {

        if (blueTeam) {
            return Commands.runOnce(() -> {
                    // Any neccessary calibration code
                    System.out.println("Balance Auto Blue Start");
            })/*.alongWith(arm.calibrateArm())*/
            .andThen(driveForwardOverChargeStationBlueCommand); // Drives forward over charge station
        } else {
            return Commands.runOnce(() -> {
                System.out.println("Balance Auto Red Start"); 
                // Any neccessary calibration code
            })/*.alongWith(arm.calibrateArm())*/
            .andThen(driveForwardOverChargeStationRedCommand); // Drives forward over charge station
        }
    }

    // Second half of balance auto
    public static CommandBase balanceAutoSecondHalf(Command driveBackwardsOntoChargeStationBlueCommand, 
            Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation) {
        
        if (blueTeam) {    
            return driveBackwardsOntoChargeStationBlueCommand
            .andThen(balanceChargeStation // Balances charge station (Runs until the end of autonomous)
            .alongWith(Commands.runOnce(() -> {
                System.out.println("Balance Auto Blue Finish");
            })));
        } else {
            return driveBackwardsOntoChargeStationRedCommand
            .andThen(balanceChargeStation // Balances charge station (Runs until the end of autonomous)
            .alongWith(Commands.runOnce(() -> {
                System.out.println("Balance Auto Red Finish");
            })));
        }
    }

    // Autonomous mode for taxi points + balancing charge station
    public static CommandBase balanceAuto(Command driveForwardOverChargeStationBlueCommand, 
            Command driveBackwardsOntoChargeStationBlueCommand, Command driveForwardOverChargeStationRedCommand, 
            Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation, ArmSubsystem arm) {

        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        return (Commands.runOnce(() -> {
            System.out.println("Balance Auto Start");
        }).andThen(balanceAutoFirstHalf(driveForwardOverChargeStationBlueCommand, driveForwardOverChargeStationRedCommand, arm)) // Drive forward over charge station
        .andThen(balanceAutoSecondHalf(driveBackwardsOntoChargeStationBlueCommand, driveBackwardsOntoChargeStationRedCommand, balanceChargeStation) // Drive backwards onto charge station and balance it continuously
        .andThen(Commands.runOnce(() -> {
            System.out.println("Balance Auto Finish"); // Shouldn't print until auton is over, if at all
        }))));
    }

    // Assumes robot is at a AprilTag
    // Might need to add calibration 
    // Places cone on top right pole
    public static CommandBase placeConeAuto(ClawGripSubsystem claw, GoTowardsCoordinatesCommandAuto goTowardsTopRight, GoTowardsCoordinatesCommandAuto goTowardsStartingPos) {
        return Commands.runOnce(() -> {
            // Any neccessary calibration code
            System.out.println("Place Cone Auto Start");
        }).andThen(goTowardsTopRight) // Arm goes to top right position on grid
        .andThen(Commands.runOnce(() -> {
            claw.setClawOpened(false); // Opens claw
        }, claw).andThen(goTowardsStartingPos)) // Returns arm to starting position
        .andThen(Commands.runOnce(() -> {
            System.out.println("Place Cone Auto Finish");
        }));
    }

    // Position values on trajectories may need to be adjusted
    // Adjustments can be made later lol
    // Might need to add calibration 
    // Places pre-loaded cone, drives backwards to pick up cube, drives forwards to place cube on grid
    public static CommandBase doublePlacementAuto(ArmSubsystem arm, ClawGripSubsystem claw, Command driveBackwardsToCubeBlue, Command driveForwardsToGridBlue, 
            Command driveBackwardsToCubeRed, Command driveForwardsToGridRed, GoTowardsCoordinatesCommandAuto goTowardsTopRight, GoTowardsCoordinatesCommandAuto goTowardsStartingPos,
            GoTowardsCoordinatesCommandAuto goTowardsStartingPos2, GoTowardsCoordinatesCommandAuto goTowardsStartingPos3, GoTowardsCoordinatesCommandAuto goTowardsPickupPos, 
            GoTowardsCoordinatesCommandAuto goTowardsTopCenter, Command aimAssist) {

        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        if (blueTeam) {
            return Commands.runOnce(() -> {
                System.out.println("Double Placement Auto Blue Start");
            }).andThen(placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos)) // Drops pre-loaded cone onto top right pole
            .andThen(driveBackwardsToCubeBlue) // Drives backwards to cube
            .andThen(Commands.runOnce(() -> { 
                claw.setClawOpened(false); // Opens claw
                NetworkTablesUtil.setLimelightPipeline(3); // Sets vision pipeline to detect cubes
            }, claw)).andThen(goTowardsPickupPos) // Arm goes to pickup position
            .andThen(aimAssist) // Guides claw to game piece
            .andThen(Commands.runOnce(() -> { 
                claw.setClawOpened(true); // Closes claw
            }, claw))
            .andThen(goTowardsStartingPos2 // Arm goes to starting position
            .alongWith(driveForwardsToGridBlue)) // Drive forwards to grid
            .andThen(goTowardsTopCenter) // Arm goes to top center position on grid
            .andThen(Commands.runOnce(() -> {
                claw.setClawOpened(false); // Opens claw
            }, claw).andThen(goTowardsStartingPos3)) // Arm goes to starting position
            .andThen(Commands.runOnce(() -> {
                System.out.println("Double Placement Auto Blue Finish");
            }));
        } else {
            return Commands.runOnce(() -> {
                System.out.println("Double Placement Auto Red Start");
            }).andThen(placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos)) // Drops pre-loaded cone onto top right pole
            .andThen(driveBackwardsToCubeRed) // Drives backwards to cube
            .andThen(Commands.runOnce(() -> { 
                claw.setClawOpened(false); // Opens claw
                NetworkTablesUtil.setLimelightPipeline(3); // Sets vision pipeline to detect cubes
            }, claw)).andThen(goTowardsPickupPos) // Arm goes to pickup position
            .andThen(aimAssist) // Guides claw to game piece
            .andThen(Commands.runOnce(() -> { 
                claw.setClawOpened(true); // Closes claw
            }, claw))
            .andThen(goTowardsStartingPos2 // Arm goes to starting position
            .alongWith(driveForwardsToGridRed)) // Drive forwards to grid
            .andThen(goTowardsTopCenter) // Arm goes to top center position on grid
            .andThen(Commands.runOnce(() -> {
                claw.setClawOpened(false); // Opens claw
            }, claw).andThen(goTowardsStartingPos3)) // Arm goes to starting position
            .andThen(Commands.runOnce(() -> {
                System.out.println("Double Placement Auto Red Finish");
            }));
        }
    }

    // Might need to add calibration
    public static CommandBase placeConeThenBalanceAuto(Command driveForwardOverChargeStationBlueCommand, Command driveBackwardsOntoChargeStationBlueCommand, 
            Command driveForwardOverChargeStationRedCommand, Command driveBackwardsOntoChargeStationRedCommand, Command balanceChargeStation, ArmSubsystem arm, ClawGripSubsystem claw,
            GoTowardsCoordinatesCommandAuto goTowardsTopRight, GoTowardsCoordinatesCommandAuto goTowardsStartingPos) {

        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        return Commands.runOnce(() -> {
            System.out.println("Place Cone then Balance Auto Start");
        }).andThen(balanceAutoFirstHalf(driveForwardOverChargeStationBlueCommand, driveForwardOverChargeStationRedCommand, arm)) // Drives forward over charge station to grid
        .andThen(placeConeAuto(claw, goTowardsTopRight, goTowardsStartingPos)) // Places pre-loaded cone on top right pole
        .andThen((balanceAutoSecondHalf(driveBackwardsOntoChargeStationBlueCommand, driveBackwardsOntoChargeStationRedCommand, balanceChargeStation)) // Drives backwards onto charge station and balances it continuously
        .andThen(Commands.runOnce(() -> {
            System.out.println("Place Cone then Balance Auto Finish"); // Shouldn't print until auton is over, if at all
        })));
    }

    // Runs double placement then balances charge station
    // Might not use, don't know if there is enough time
    public static CommandBase doublePlacementThenBalanceAuto(ArmSubsystem arm, ClawGripSubsystem claw, Command driveBackwardsToCubeBlue, Command driveForwardsToGridBlue, 
            Command driveBackwardsToCubeRed, Command driveForwardsToGridRed, GoTowardsCoordinatesCommandAuto goTowardsTopRight, GoTowardsCoordinatesCommandAuto goTowardsStartingPos,
            GoTowardsCoordinatesCommandAuto goTowardsStartingPos2, GoTowardsCoordinatesCommandAuto goTowardsStartingPos3, GoTowardsCoordinatesCommandAuto goTowardsPickupPos, 
            GoTowardsCoordinatesCommandAuto goTowardsTopCenter, Command driveBackwardsOntoChargeStationDPBlue, Command driveBackwardsOntoChargeStationDPRed, Command balanceCommand, Command aimAssist) {
        
        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        return Commands.runOnce(() -> {
            System.out.println("Double Placement then Balance Auto Start");
        }).andThen(doublePlacementAuto(arm, claw, driveBackwardsToCubeBlue, driveForwardsToGridBlue, driveBackwardsToCubeRed, driveForwardsToGridRed, goTowardsTopRight, 
                goTowardsStartingPos, goTowardsStartingPos2, goTowardsStartingPos3, goTowardsPickupPos, goTowardsTopCenter, aimAssist)) // Runs double placement command
        .andThen(blueTeam ? driveBackwardsOntoChargeStationDPBlue : driveBackwardsOntoChargeStationDPRed) // Drives backwards onto charge station
        .andThen(balanceCommand) // Balances the charge station continuously
        .andThen(Commands.runOnce(() -> {
            System.out.println("Double Placement then Balance Auto Finish"); // Shouldn't print until auton is over, if at all
        }));
    }

    private Autos() {
        throw new UnsupportedOperationException("Autos is a utility class and cannot be instantiated!");
    }
}
