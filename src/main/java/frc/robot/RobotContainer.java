// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.commands.armcommands.ArmControlCommand;
import frc.robot.commands.armcommands.ArmTestCommand;
import frc.robot.commands.clawcommands.ClawOpenandCloseCommand;
import frc.robot.commands.clawcommands.ClawRotateCommand;
import frc.robot.commands.drivecommands.BalanceChargeStationCommand;
import frc.robot.commands.drivecommands.ManualDriveCommand;
import frc.robot.controllers.FlightJoystick;
import frc.robot.commands.autocommands.Autos;
import frc.robot.controllers.XboxController;
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
    public final XboxController xboxController = new XboxController(new CommandXboxController(OperatorConstants.XBOX_CONTROLLER_PORT));

    // The robot's subsystems and commands are defined here...
    public final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(driverController);
    public final ArmSubsystem arm = new ArmSubsystem();
    public final ClawGripSubsystem clawGrip = new ClawGripSubsystem();
    public final ClawRotationSubsystem clawRotation = new ClawRotationSubsystem();

    public final TrajectoryReader trajectoryReader = new TrajectoryReader("robogui", "trajectory");

    public final ManualDriveCommand manualDrive = new ManualDriveCommand(driveTrain, driverController);
    public final BalanceChargeStationCommand balanceCommand = new BalanceChargeStationCommand(driveTrain);

    // these ones got changed to xbox
    public final ArmTestCommand testArmControl = new ArmTestCommand(arm, xboxController);
    public final ArmControlCommand armControl = new ArmControlCommand(arm, xboxController);
    public final ClawOpenandCloseCommand clawOpenandCloseCommand = new ClawOpenandCloseCommand(clawGrip, xboxController);
    public final ClawRotateCommand clawRotateCommand = new ClawRotateCommand(clawRotation, xboxController);
    // end change to xbox

    /*private String trajectory1JSON = "paths/YourPath.wpilib.json"; // placeholder
    //private Trajectory autonTrajectory1 = new Trajectory(); // placeholder
    public Command trajectory1Command;*/

    private final Command defaultAuto = Autos.defaultAuto(/* pass in parameters */); // placeholder, pass in subsystems or commands if needed
    private final Command customAuto = Autos.exampleAuto(/*pass in parameters */);   // placeholder, pass in subsystems or commands if needed
    private final Command placeConeCommandAuto = Autos.armPlaceConeAuto(arm, clawGrip);

    private Command m_autonomousCommand;
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

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
       
        // armController.joystick.button(ControllerConstants.CLAW_ROTATE_RIGHT_BUTTON_NUMBER).whileTrue(clawRotation.rotateClawRight());
        // armController.joystick.button(ControllerConstants.CLAW_ROTATE_LEFT_BUTTON_NUMBER).whileTrue(clawRotation.rotateClawLeft());
        // armController.joystick.button(ControllerConstants.AUTO_ROTATE_BUTTON_NUMBER).whileTrue(clawRotation.autoRotate());
        xboxController.controller.button(ControllerConstants.CALIBRATE_ARM_BUTTON_NUMBER).onTrue(arm.calibrateArm());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_chooser.getSelected();
    }

    public void onRobotInit() {
        m_chooser.setDefaultOption("Default Auto", defaultAuto);
        m_chooser.addOption("My Auto", customAuto);
        m_chooser.addOption("woohoo arm place cone yay", placeConeCommandAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        /*try {
            Path trajectory1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
            autonTrajectory1 = TrajectoryUtil.fromPathweaverJson(trajectory1Path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectory1JSON, ex.getStackTrace());
        }

        trajectory1Command = driveTrain.followTrajectoryCommand(autonTrajectory1);*/
    }

    public void onAutonInit() {
        inTeleop = false;
        
        m_autonomousCommand = this.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    public void onTeleopInit() {
        inTeleop = true;
        this.arm.setPIDControlState(false);
        

        this.driveTrain.setDefaultCommand(this.manualDrive);
        //this.arm.setDefaultCommand(this.testArmControl);
        this.arm.setDefaultCommand(this.armControl);
        this.clawGrip.setDefaultCommand(this.clawOpenandCloseCommand);
        this.clawRotation.setDefaultCommand(this.clawRotateCommand);
    }
}
