// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;


public final class Autos {
    /**
     * Example static factory for an autonomous command.
     */
    public static CommandBase exampleAuto() {
        // Example command case
        return Commands.runOnce(() -> {
            // Autonomous scenario codee
        });

    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
