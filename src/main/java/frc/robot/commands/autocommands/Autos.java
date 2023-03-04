// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */
    public static CommandBase exampleAuto(/* parameter of type Subsystem goes here (you can have multiple), make sure to pass them in when the method is called inside RobotContainer */) {
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario code
            
        }) /* susbsystems that are used inside the curly braces above must be passed in here */;

        // To implement a sequence of actions/commands use .andThen(), can be used to implement PathWeaver trajectories
        // To implement simultaneous actions/commands use .alongWith(), can also be used to implement PathWeaver trajectories
        // For example: return Commands.runOnce(() -> {}).andThen(RobotContainer.trajectory1Command);
        // Example of simultaneous implementation: return Commands.runOnce(() -> {}).alongWith(RobotContainer.trajectory1Command);
    }

    public static CommandBase defaultAuto() {
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario code
        });

    }

    private Autos() {
        throw new UnsupportedOperationException("Autos is a utility class and cannot be instantiated!");
    }
}
