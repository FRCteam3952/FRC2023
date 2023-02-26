// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.armcommands.ArmControlCommand;
import frc.robot.commands.armcommands.ArmTestCommand;
import frc.robot.commands.armcommands.AutomaticObjectPlacementCommand;
import frc.robot.commands.clawcommands.ClawOpenandCloseCommand;
import frc.robot.commands.clawcommands.ClawRotateCommand;
import frc.robot.joystick.FlightJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.ArmGyro;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.wrappers.TrajectoryReader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    public static boolean inTeleop = false;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final FlightJoystick driverController = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    public final FlightJoystick armController = new FlightJoystick(new CommandJoystick(OperatorConstants.LEFT_JOYSTICK_PORT));

    // The robot's subsystems and commands are defined here...
    public final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(driverController);
    public final ArmSubsystem arm = new ArmSubsystem();
    public final ClawGripSubsystem clawGrip = new ClawGripSubsystem();
    public final ClawRotationSubsystem clawRotation = new ClawRotationSubsystem();

    public final TrajectoryReader trajectoryReader = new TrajectoryReader("robogui", "trajectory");

    public final ManualDriveCommand manualDrive = new ManualDriveCommand(driveTrain, driverController);

    public final ArmTestCommand testArmControl = new ArmTestCommand(arm, armController);
    public final ArmControlCommand armControl = new ArmControlCommand(arm, armController);
    public final AutomaticObjectPlacementCommand autoObjectPlacement = new AutomaticObjectPlacementCommand(arm, armController);
    public final ClawOpenandCloseCommand clawOpenandCloseCommand = new ClawOpenandCloseCommand(clawGrip, armController);
    public final ClawRotateCommand clawRotateCommand = new ClawRotateCommand(clawRotation, armController);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        // Poke the static classes so their static initializers are run at startup.
        LimeLight.poke();
        RobotGyro.poke();
        ArmGyro.poke();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(exampleSubsystem::exampleCondition)
        //    .onTrue(new ManualDrive(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

        // driverController.joystick.button(ControllerConstants.RUN_GUI_TRAJECTORY_BUTTON_NUMBER).onTrue(this.driveTrain.followTrajectoryCommand(this.trajectoryReader.currentTrajectory));
        /*
        armController.joystick.button(ControllerConstants.CLAW_GRIP_BUTTON_NUMBER)
            .and(armController.joystick.button(ControllerConstants.CLAW_RELEASE_BUTTON_NUMBER).negate())
            .onFalse(clawGrip.stopClaw());
        armController.joystick.button(ControllerConstants.CLAW_RELEASE_BUTTON_NUMBER)
            .and(armController.joystick.button(ControllerConstants.CLAW_GRIP_BUTTON_NUMBER).negate())
            .onFalse(clawGrip.stopClaw());
        armController.joystick.button(ControllerConstants.CLAW_GRIP_BUTTON_NUMBER).whileTrue(clawGrip.closeClaw());
        armController.joystick.button(ControllerConstants.CLAW_RELEASE_BUTTON_NUMBER).whileTrue(clawGrip.openClaw());
        */
        // armController.joystick.button(ControllerConstants.CLAW_ROTATE_RIGHT_BUTTON_NUMBER).whileTrue(clawRotation.rotateClawRight());
        // armController.joystick.button(ControllerConstants.CLAW_ROTATE_LEFT_BUTTON_NUMBER).whileTrue(clawRotation.rotateClawLeft());
        // armController.joystick.button(ControllerConstants.AUTO_ROTATE_BUTTON_NUMBER).whileTrue(clawRotation.autoRotate());
        //armController.joystick.button(ControllerConstants.CALIBRATE_ARM_BUTTON_NUMBER).onTrue(arm.calibrateArm());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Commands.none();// Autos.exampleAuto(exampleSubsystem);
    }

    public void onAutonInit() {
        inTeleop = false;
    }

    public void onTeleopInit() {
        inTeleop = true;

        this.driveTrain.setDefaultCommand(this.manualDrive);
        this.arm.setDefaultCommand(this.testArmControl);
        this.clawGrip.setDefaultCommand(this.clawOpenandCloseCommand);
    }
}
