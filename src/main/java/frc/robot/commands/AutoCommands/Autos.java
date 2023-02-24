// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;


public final class Autos {
    private final static DriveTrainSubsystem drive = new DriveTrainSubsystem();
    private final static ArmSubsystem arm = new ArmSubsystem();

    /**
     * Example static factory for an autonomous command.
     */
    public static CommandBase exampleAuto() {
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario codee
        }, drive, arm);

    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
