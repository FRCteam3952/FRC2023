// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
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
    // Robot drives backwards
    public static CommandBase taxiAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> {
                System.out.println("Taxi Auto Start");
            }
        )
        .andThen(
            resetTimerCommand()
        )
        .andThen(
            Commands.run(
                () -> {      
                    if (timer.get() < 6) {
                        System.out.println("Slow Drive");
                        robot.driveTrain.tankDrive(0.25, 0); // Drives backwards slowly to edge of charge station for 1.15 seconds
                    } else {
                        robot.driveTrain.tankDrive(0, 0);
                        System.out.println("Taxi Auto Finish");          
                    }
                }, 
                robot.driveTrain
            )
        );
    }

    // Robot drives backwards then forwards onto charge station
    public static CommandBase taxiForBalanceAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> {
                System.out.println("Taxi For Balance Auto Start");
            }
        )
        .andThen(
            resetTimerCommand()
        )
        .andThen(
            Commands.run(
                () -> {      
                    if (timer.get() < 1.15) {
                        System.out.println("Slow Drive Backwards");
                        robot.driveTrain.tankDrive(0.25, 0); // Drives backwards slowly to edge of charge station for 1.15 seconds
                    } else if (timer.get() < 3.45) {
                        System.out.println("Fast Drive Backwards");
                        robot.driveTrain.tankDrive(0.5, 0); // Drives backwards faster over charge station for 2.5 seconds
                    } else if (timer.get() < 4.90) {
                        System.out.println("Fast Drive Forwards");
                        robot.driveTrain.tankDrive(-0.5, 0); // Drives forwards onto charge station for 1.5 seconds
                    } else {
                        robot.driveTrain.tankDrive(0, 0);
                        System.out.println("Taxi For Balance Auto Finish");          
                    }
                }, 
                robot.driveTrain
            )
        );
    }

    // Might want to test later if we have time
    // Robot drives backwards then forwards onto charge station based off on gyro pitch
    public static CommandBase dynamicTaxiForBalanceAuto(RobotContainer robot) {
        // TODO make sure pitch isn't broken
        return Commands.runOnce(
            () -> {
                System.out.println("Dynamic Taxi For Balance Auto Start ");
                RobotGyro.resetGyroAngle();
            }
        )
        .andThen( resetTimerCommand() )
        .andThen(
                Commands.run(
                () -> {// Drive until the robot is on the far edge of the charge station
                    robot.driveTrain.tankDrive(0.5, 0);
                }, 
                robot.driveTrain
            )
        )
        .until(
            () -> RobotGyro.getGyroAngleDegreesPitch() > 8 || timer.get() > 2.8 // TODO CHECK TIMER VALUES
        ) 
        .andThen(
            Commands.run(
                () -> { // Drive until the robot is past the charge station and level
                    robot.driveTrain.tankDrive(0.5, 0);
                }, 
                robot.driveTrain
            )
        )
        .until(
            () -> (Math.abs(RobotGyro.getGyroAngleDegreesPitch()) < 2 || timer.get() > 2.8) // TODO CHECK TIMER VALUES
        )
        .andThen( resetTimerCommand() )
        .andThen(
            Commands.run(
                () -> { // Drive until the robot is at the far edge again
                    robot.driveTrain.tankDrive(-0.5, 0);
                }, 
                robot.driveTrain
            )
        )
        .until(
            () -> (Math.abs(RobotGyro.getGyroAngleDegreesPitch()) > 8 || timer.get() > 1) // TODO CHECK TIMER VALUES
        )
        .andThen(
            Commands.run(
                () -> { // Drive until in the center of the charge station
                    robot.driveTrain.tankDrive(-0.5, 0);
                }, 
                robot.driveTrain
            )
        )
        .until(
            () -> (Math.abs(RobotGyro.getGyroAngleDegreesPitch()) < 2 || timer.get() > 1) // TODO CHECK TIMER VALUES
        )
        .andThen(
            Commands.run(
                () -> {
                    robot.driveTrain.tankDrive(0, 0); // stop
                }, 
                robot.driveTrain
            )
        );
    }

    // Places cube on top center platform
    public static CommandBase placeCubeAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> { // Closes the claw around pre-loaded cube
                System.out.println("Place Cube Auto Start");
                robot.clawGrip.setClawOpened(false); // Closes claw
            }, 
            robot.clawGrip
        )
        .andThen(robot.goToTopCenter.get()) // Moves arm to top center position on grid to place cube
        .andThen( waitCommand(0.2) ) // Waits 0.5 seconds
        .andThen(
            Commands.runOnce(
                () -> { // Opens claw to drop pre-loaded cube onto top center platform
                    System.out.println("Place Cube Auto Running");
                    robot.clawGrip.setClawOpened(true); // Opens claw
                }, 
                robot.clawGrip
            )
        )
        .andThen( waitCommand(0.2) ) // Waits 0.5 seconds
        .andThen(robot.goToStartingPos.get()); // Moves arm to starting position
    }

    // Places cube on top center platform then runs taxi
    public static CommandBase placeCubeThenTaxiAuto(RobotContainer robot) {
        return placeGamePieceAuto(robot) // Places cube on top center grid position
        .andThen(taxiAuto(robot)); // Initiates taxi (drives backwards)
    }

    // Places cube on top center platform
    public static CommandBase placeGamePieceAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> { // Closes the claw around pre-loaded cube
                System.out.println("Place Game Piece Auto Start");
                robot.clawGrip.setClawOpened(false); // Closes claw
            }, 
            robot.clawGrip
        )
        .andThen(robot.goToTopCenter.get()) // Moves arm to top center position on grid to place cube
        .andThen(waitCommand(0.2)) // Waits 0.2 seconds
        .andThen(
            Commands.runOnce(
                () -> { // Opens claw to drop pre-loaded cube onto top center platform
                    System.out.println("Place Game Piece Auto Running");
                    robot.clawGrip.setClawOpened(true); // Opens claw
                }, 
                robot.clawGrip
            )
        )
        .andThen(waitCommand(0.2)) // Waits 0.2 seconds
        .andThen(robot.goToStartingPos.get()); // Moves arm to starting position
    }

    // Runs taxi for balance then balances charge station
    public static CommandBase taxiThenBalanceAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> {
                System.out.println("Taxi Auto then Balance Start");
            }
        )
        .andThen( resetTimerCommand() ) // I just added this while reformatting, it seems like it should be here
        .andThen(
            taxiForBalanceAuto(robot)
            .until(() -> timer.get() > 5) // reformatting, does this need to be in the andThen block?
        )
        .andThen(robot.balanceCommand.get());
    }

    // Double placement: places cube on top center platform, drives backwards to pick up cone, drives forward towards grid, places cone on top right pole
    public static CommandBase placeCubeThenConeAuto(RobotContainer robot) {
        return placeCubeAuto(robot) // Places cube on top center section of grid
        .andThen( resetTimerCommand() ) // Resets timer
        .andThen(
            Commands.run(
                () -> {
                    // driveTrain.tankDrive(0.25, 0); // Drives backwards for 4.25 seconds to pick up cone
                }, 
                robot.driveTrain
            )
            .until(() -> timer.get() > 4.25)
            .andThen(
                Commands.runOnce(
                    () -> {
                        robot.driveTrain.tankDrive(0, 0);
                    }, 
                    robot.driveTrain
                )
            )
            .alongWith(robot.goToAbovePickupPos.get()) // Goes to 10 inches above pickup position
        ) 
        .andThen(
            Commands.runOnce(
                () -> {
                    NetworkTablesUtil.setLimelightPipeline(1); // Changes pipeline to detect cones
                }
            )
        )
        .andThen(robot.aimAssist.get()) // Guides claw to game piece
        .andThen(robot.goToPickupPosX30.get()) // Goes to pickup position
        .andThen( waitCommand(0.5) ) // Waits 0.5 seconds
        .andThen(
            Commands.runOnce(
                () -> { // Closes claw around game piece
                    System.out.println("Place Cube then Cone Auto Running");
                    robot.clawGrip.setClawOpened(false); // Closes claw
                }, 
                robot.clawGrip
            )
        )
        .andThen( waitCommand(0.5) ) // Waits 0.5 seconds
        .andThen(
            robot.goToStartingPos.get() // Arm goes to starting position
            .alongWith(
                resetTimerCommand() // Resets timer
                .andThen(
                    Commands.run(
                        () -> {
                        // driveTrain.tankDrive(-0.25, 0); // Drives forwards for 4.25 seconds towards grid
                    }, 
                    robot.driveTrain
                )
                .until( () -> timer.get() > 4.25) )
            )
        )
        .andThen(
            Commands.runOnce(
                () -> {
                    robot.driveTrain.tankDrive(0, 0);
                }, 
                robot.driveTrain
            )
        )
        .andThen(robot.goToTopRight.get()); // Arm goes to top right pole to place cone
    }

    public static CommandBase placeCubeThenTaxiThenBalanceAuto(RobotContainer robot) {
        return placeGamePieceAuto(robot)
        .andThen(taxiThenBalanceAuto(robot));
    }

    // Has the robot do nothing for a set time (in seconds)
    public static CommandBase waitCommand(double seconds) {
        return resetTimerCommand()
        .andThen(
            Commands.run(
                () -> {
                    System.out.println("Waiting for " + seconds + " seconds | " + timer.get());
                }
            )
            .until( () -> timer.get() > seconds )
        );
    }

    // Resets the timer
    public static CommandBase resetTimerCommand() {
        return Commands.runOnce(() -> {
            timer.reset();
            timer.start();
        });
    }

    /*
     * EVERYTHING BELOW USES PATHWEAVER TRAJECTORIES, WHICH CURRENTLY MIGHT WORK
     * TRY TO GET THEM WORKING BEFORE LA REGIONALS, BUT IS NOW PRETTY IMPORTANT
     * THERES A LOT OF THINGS THAT NEED TO BE ADJUSTED (STILL TRUE)
     */

    public static CommandBase moveOneMeter(Command moveOneMeter) {
        return (CommandBase) moveOneMeter;
    }


    // First half of balance auto
    public static CommandBase balanceAutoFirstHalf(RobotContainer robot) {

        if (blueTeam) {
            return Commands.runOnce(
                () -> {
                        // Any neccessary calibration code
                        System.out.println("Balance Auto Blue Start");
                }
            )/*.alongWith(arm.calibrateArm())*/
            .andThen(robot.driveForwardOverChargeStationBlue.get()); // Drives forward over charge station

        } else {
            return Commands.runOnce(
                () -> {
                    System.out.println("Balance Auto Red Start"); 
                    // Any neccessary calibration code
                }
            )/*.alongWith(arm.calibrateArm())*/
            .andThen(robot.driveForwardOverChargeStationRed.get()); // Drives forward over charge station
        }
    }

    // Second half of balance auto
    public static CommandBase balanceAutoSecondHalf(RobotContainer robot) {
        
        if (blueTeam) {    
            return robot.driveBackwardsOntoChargeStationBlue.get()
            .andThen(
                robot.balanceCommand.get() // Balances charge station (Runs until the end of autonomous)
                .alongWith(
                    Commands.runOnce(
                        () -> {
                            System.out.println("Balance Auto Blue Finish");
                        }
                    )
                )
            );
            
        } else {
            return robot.driveBackwardsOntoChargeStationRed.get()
                .andThen(
                    robot.balanceCommand.get() // Balances charge station (Runs until the end of autonomous)
                    .alongWith(
                        Commands.runOnce(
                            () -> {
                                System.out.println("Balance Auto Red Finish");
                            }
                        )
                    )
                );
        }
    }

    // Autonomous mode for taxi points + balancing charge station
    public static CommandBase balanceAuto(RobotContainer robot) {

        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        return Commands.runOnce(
                () -> {
                    System.out.println("Balance Auto Start");
                }
            )
            .andThen(
                balanceAutoFirstHalf(robot) // Drive forward over charge station
            ) 
            .andThen(
                balanceAutoSecondHalf(robot) // Drive backwards onto charge station and balance it continuously
            ) 
            .andThen(
                Commands.runOnce(
                    () -> {
                        System.out.println("Balance Auto Finish"); // Shouldn't print until auton is over, if at all
                    }
                )
            );
    }

    // Assumes robot is at a AprilTag
    // Might need to add calibration 
    // Places cone on top right pole
    public static CommandBase placeConeAuto(RobotContainer robot) {
        return Commands.runOnce(
            () -> {
                // Any neccessary calibration code
                System.out.println("Place Cone Auto Start");
            }
        )
        .andThen(robot.goToTopRight.get()) // Arm goes to top right position on grid
        .andThen(
            Commands.runOnce(
                () -> {
                    robot.clawGrip.setClawOpened(false); // Opens claw
                }, 
                robot.clawGrip
            )
            .andThen(robot.goToStartingPos.get())
        ) // Returns arm to starting position
        .andThen(
            Commands.runOnce(
                () -> {
                    System.out.println("Place Cone Auto Finish");
                }
            )
        );
    }

    // Position values on trajectories may need to be adjusted
    // Adjustments can be made later lol
    // Might need to add calibration 
    // Places pre-loaded cone, drives backwards to pick up cube, drives forwards to place cube on grid
    public static CommandBase doublePlacementAuto(RobotContainer robot) {
        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        // TODO someone make this code into smaller chunks or something
        if (blueTeam) {
            return Commands.runOnce(
                () -> {
                    System.out.println("Double Placement Auto Blue Start");
                }
            )
            .andThen(placeGamePieceAuto(robot)) // Drops pre-loaded cube onto top center platform
            .andThen(robot.driveBackwardsToConeBlue.get() // Drives backwards to cone
            .alongWith(robot.goToAbovePickupPos.get())) // Goes to 10 inches above pickup position
            .andThen(
                Commands.runOnce(
                    () -> {
                        NetworkTablesUtil.setLimelightPipeline(1); // Changes pipeline to detect cones
                    }
                )
            )
            .andThen(robot.aimAssist.get()) // Guides claw to game piece
            .andThen(robot.goToPickupPosX30.get()) // Goes to pickup position
            .andThen(waitCommand(0.5)) // Waits 0.5 seconds
            .andThen(
                Commands.runOnce(
                    () -> { // Closes claw around game piece
                        System.out.println("Double Placement Auto Blue Running");
                        robot.clawGrip.setClawOpened(false); // Closes claw
                    }, 
                    robot.clawGrip
                )
            )
            .andThen(waitCommand(0.5)) // Waits 0.5 seconds
            .andThen(
                robot.goToStartingPos.get() // Arm goes to starting position
                .alongWith(robot.driveForwardsToGridBlue.get())
            ) // Drive forwards to grid
            .andThen(placeGamePieceAuto(robot)) // Drops cone onto top right pole
            .andThen(
                Commands.runOnce(
                    () -> {
                        System.out.println("Double Placement Auto Blue Finish");
                    }
                )
            );

        } else {
            return Commands.runOnce(
                () -> {
                    System.out.println("Double Placement Auto Red Start");
                }
            )
            .andThen(placeGamePieceAuto(robot)) // Drops pre-loaded cube onto top center platform
            .andThen(
                robot.driveBackwardsToConeRed.get() // Drives backwards to cone
                .alongWith(robot.goToAbovePickupPos.get()) // Goes to 10 inches above pickup position
            ) 
            .andThen(
                Commands.runOnce(
                    () -> {
                        NetworkTablesUtil.setLimelightPipeline(1); // Changes pipeline to detect cones
                    }
                )
            )
            .andThen(robot.aimAssist.get()) // Guides claw to game piece
            .andThen(robot.goToPickupPosX30.get()) // Goes to pickup position
            .andThen(waitCommand(0.5)) // Waits 0.5 seconds
            .andThen(
                Commands.runOnce(
                    () -> { // Closes claw around game piece
                        System.out.println("Double Placement Auto Red Running");
                        robot.clawGrip.setClawOpened(false); // Closes claw
                    }, 
                    robot.clawGrip
                )
            )
            .andThen(waitCommand(0.5)) // Waits 0.5 seconds
            .andThen(
                robot.goToStartingPos.get() // Arm goes to starting position
                .alongWith(robot.driveForwardsToGridRed.get()) // Drive forwards to grid
            )
            .andThen(placeGamePieceAuto(robot)) // Drops cone onto top left pole
            .andThen(
                Commands.runOnce(
                    () -> {
                        System.out.println("Double Placement Auto Red Finish");
                    }
                )
            );
        }
    }

    // Might need to add calibration
    public static CommandBase placeConeThenBalanceAuto(RobotContainer robot) {

        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();

        return Commands.runOnce(
                () -> {
                    System.out.println("Place Cone then Balance Auto Start");
                }
            )
            .andThen(balanceAutoFirstHalf(robot)) // Drives forward over charge station to grid
            .andThen(placeConeAuto(robot)) // Places pre-loaded cone on top right pole
            .andThen(
                balanceAutoSecondHalf(robot) // Drives backwards onto charge station and balances it continuously
                .andThen(
                    Commands.runOnce(
                        () -> {
                            System.out.println("Place Cone then Balance Auto Finish"); // Shouldn't print until auton is over, if at all
                        }
                    )
                )
            );
    }

    

    // Runs double placement then balances charge station
    // Might not use, don't know if there is enough time
    public static CommandBase doublePlacementThenBalanceAuto(RobotContainer robot) {
        
        blueTeam = NetworkTablesUtil.getIfOnBlueTeam();
        return Commands.runOnce(
                () -> {
                    System.out.println("Double Placement then Balance Auto Start");
                }
            )
            .andThen(doublePlacementAuto(robot)) // Runs double placement command
            .andThen(blueTeam ? robot.driveBackwardsOntoChargeStationDPBlue.get() : robot.driveBackwardsOntoChargeStationDPRed.get()) // Drives backwards onto charge station
            .andThen(robot.balanceCommand.get()) // Balances the charge station continuously
            .andThen(
                Commands.runOnce(
                    () -> {
                        System.out.println("Double Placement then Balance Auto Finish"); // Shouldn't print until auton is over, if at all
                    }
                )
            );
    }

    private Autos() {
        throw new UnsupportedOperationException("Autos is a utility class and cannot be instantiated!");
    }
}
