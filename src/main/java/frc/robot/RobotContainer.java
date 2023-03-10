// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import javax.swing.text.Position;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.commands.armcommands.ArmControlCommand;
import frc.robot.commands.armcommands.ArmTestCommand;
import frc.robot.commands.armcommands.GoTowardsCoordinatesCommand;
import frc.robot.commands.autocommands.Autos;
import frc.robot.commands.clawcommands.ClawOpenandCloseCommand;
import frc.robot.commands.clawcommands.ClawRotateCommand;
import frc.robot.commands.drivecommands.BalanceChargeStationCommand;
import frc.robot.commands.drivecommands.ManualDriveCommand;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawGripSubsystem;
import frc.robot.subsystems.ClawRotationSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.MPU6050;
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
    public final BalanceChargeStationCommand balanceCommand2 = new BalanceChargeStationCommand(driveTrain); // Only here because compositions can't use commands that have already been used for other compositions

    // these ones got changed to xbox
    public final ArmTestCommand testArmControl = new ArmTestCommand(arm, xboxController);
    public final ArmControlCommand armControl = new ArmControlCommand(arm, xboxController);
    public final GoTowardsCoordinatesCommand goTowardsTopRight = new GoTowardsCoordinatesCommand(arm, PositionConstants.TOP_RIGHT_POS);
    public final GoTowardsCoordinatesCommand goTowardsTopRight2 = new GoTowardsCoordinatesCommand(arm, PositionConstants.TOP_RIGHT_POS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsTopRight3 = new GoTowardsCoordinatesCommand(arm, PositionConstants.TOP_RIGHT_POS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsTopCenter = new GoTowardsCoordinatesCommand(arm, PositionConstants.TOP_CENTER_POS);
    public final GoTowardsCoordinatesCommand goTowardsStartingPos = new GoTowardsCoordinatesCommand(arm, ArmConstants.STARTING_COORDS);
    public final GoTowardsCoordinatesCommand goTowardsStartingPos2 = new GoTowardsCoordinatesCommand(arm, ArmConstants.STARTING_COORDS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsStartingPos3 = new GoTowardsCoordinatesCommand(arm, ArmConstants.STARTING_COORDS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsStartingPos4 = new GoTowardsCoordinatesCommand(arm, ArmConstants.STARTING_COORDS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsStartingPos5 = new GoTowardsCoordinatesCommand(arm, ArmConstants.STARTING_COORDS); // Only here because compositions can't use commands that have already been used for other compositions
    public final GoTowardsCoordinatesCommand goTowardsPickupPos = new GoTowardsCoordinatesCommand(arm, new double[] {-30, ArmConstants.PICK_UP_POSITION_Y, 0});

    public final ClawOpenandCloseCommand clawOpenandCloseCommand = new ClawOpenandCloseCommand(clawGrip, xboxController);
    public final ClawRotateCommand clawRotateCommand = new ClawRotateCommand(clawRotation, xboxController);
    // end change to xbox

    private String driveForwardOverChargeStationBlueJSON = "paths/DriveForwardOverChargeStationBlue.wpilib.json"; 
    private Trajectory driveForwardOverChargeStationBlueTraj = new Trajectory(); 
    public Command driveForwardOverChargeStationBlueCommand;
    public Command driveForwardOverChargeStationBlueCommand2; // Only here because compositions can't use commands that have already been used for other compositions
    private String driveBackwardsOntoChargeStationBlueJSON = "paths/DriveBackwardsOntoChargeStationBlue.wpilib.json";
    private Trajectory driveBackwardsOntoChargeStationBlueTraj = new Trajectory();
    public Command driveBackwardsOntoChargeStationBlueCommand;
    public Command driveBackwardsOntoChargeStationBlueCommand2; // Only here because compositions can't use commands that have already been used for other compositions
    private String driveForwardOverChargeStationRedJSON = "paths/DriveForwardOverChargeStationRed.wpilib.json"; 
    private Trajectory driveForwardOverChargeStationRedTraj = new Trajectory(); 
    public Command driveForwardOverChargeStationRedCommand;
    public Command driveForwardOverChargeStationRedCommand2; // Only here because compositions can't use commands that have already been used for other compositions
    private String driveBackwardsOntoChargeStationRedJSON = "paths/DriveBackwardsOntoChargeStationRed.wpilib.json";
    private Trajectory driveBackwardsOntoChargeStationRedTraj = new Trajectory();
    public Command driveBackwardsOntoChargeStationRedCommand;
    public Command driveBackwardsOntoChargeStationRedCommand2; // Only here because compositions can't use commands that have already been used for other compositions

    public String driveBackwardsToCubeBlueJSON = "paths/DriveBackwardsToCubeBlue.wpilib.json";
    public Trajectory driveBackwardsToCubeBlueTraj = new Trajectory();
    public Command driveBackwardsToCubeBlueCommand;
    public String driveForwardsToGridBlueJSON = "paths/DriveForwardsToGridBlue.wpilib.json";
    public Trajectory driveForwardsToGridBlueTraj = new Trajectory();
    public Command driveForwardsToGridBlueCommand;
    public String driveBackwardsToCubeRedJSON = "paths/DriveBackwardsToCubeRed.wpilib.json";
    public Trajectory driveBackwardsToCubeRedTraj = new Trajectory();
    public Command driveBackwardsToCubeRedCommand;
    public String driveForwardsToGridRedJSON = "paths/DriveForwardsToGridRed.wpilib.json";
    public Trajectory driveForwardsToGridRedTraj = new Trajectory();
    public Command driveForwardsToGridRedCommand;

    private Command defaultAuto = Autos.defaultAuto(/* pass in parameters */); // placeholder, pass in subsystems or commands if needed
    private Command customAuto = Autos.exampleAuto(/*pass in parameters */);   // placeholder, pass in subsystems or commands if needed
    private Command placeConeCommandAuto;
    private Command balanceChargeStationAuto; 
    private Command doublePlacementAuto;
    private Command placeConeThenBalanceAuto;
    
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
        MPU6050.poke();
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

        // driverController.joystick.button(ControllerConstants.RUN_GUI_TRAJECTORY_BUTTON_NUMBER).onTrue(this.driveTrain.generateRamseteCommand(this.trajectoryReader.currentTrajectory));
       
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
        /**
         * Initialize Pathweaver trajectories/commands here
         */
        try {
            Path driveForwardOverChargeStationBluePath = Filesystem.getDeployDirectory().toPath().resolve(driveForwardOverChargeStationBlueJSON);
            driveForwardOverChargeStationBlueTraj = TrajectoryUtil.fromPathweaverJson(driveForwardOverChargeStationBluePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveForwardOverChargeStationBlueJSON, ex.getStackTrace());
        }

        try {
            Path driveBackwardsOntoChargeStationBluePath = Filesystem.getDeployDirectory().toPath().resolve(driveBackwardsOntoChargeStationBlueJSON);
            driveBackwardsOntoChargeStationBlueTraj = TrajectoryUtil.fromPathweaverJson(driveBackwardsOntoChargeStationBluePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveBackwardsOntoChargeStationBlueJSON, ex.getStackTrace());
        }

        try {
            Path driveForwardOverChargeStationRedPath = Filesystem.getDeployDirectory().toPath().resolve(driveForwardOverChargeStationRedJSON);
            driveForwardOverChargeStationRedTraj = TrajectoryUtil.fromPathweaverJson(driveForwardOverChargeStationRedPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveForwardOverChargeStationRedJSON, ex.getStackTrace());
        }

        try {
            Path driveBackwardsOntoChargeStationRedPath = Filesystem.getDeployDirectory().toPath().resolve(driveBackwardsOntoChargeStationRedJSON);
            driveBackwardsOntoChargeStationRedTraj = TrajectoryUtil.fromPathweaverJson(driveBackwardsOntoChargeStationRedPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveBackwardsOntoChargeStationRedJSON, ex.getStackTrace());
        }

        try {
            Path driveBackwardsToCubeBluePath = Filesystem.getDeployDirectory().toPath().resolve(driveBackwardsToCubeBlueJSON);
            driveBackwardsToCubeBlueTraj = TrajectoryUtil.fromPathweaverJson(driveBackwardsToCubeBluePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveBackwardsToCubeBlueJSON, ex.getStackTrace());
        }

        try {
            Path driveForwardsToGridBluePath = Filesystem.getDeployDirectory().toPath().resolve(driveForwardsToGridBlueJSON);
            driveForwardsToGridBlueTraj = TrajectoryUtil.fromPathweaverJson(driveForwardsToGridBluePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveForwardsToGridBlueJSON, ex.getStackTrace());
        }

        try {
            Path driveBackwardsToCubeRedPath = Filesystem.getDeployDirectory().toPath().resolve(driveBackwardsToCubeRedJSON);
            driveBackwardsToCubeRedTraj = TrajectoryUtil.fromPathweaverJson(driveBackwardsToCubeRedPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveBackwardsToCubeRedJSON, ex.getStackTrace());
        }

        try {
            Path driveForwardsToGridRedPath = Filesystem.getDeployDirectory().toPath().resolve(driveForwardsToGridRedJSON);
            driveForwardsToGridRedTraj = TrajectoryUtil.fromPathweaverJson(driveForwardsToGridRedPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + driveForwardsToGridRedJSON, ex.getStackTrace());
        }

        driveForwardOverChargeStationBlueCommand = driveTrain.generateRamseteCommand(driveForwardOverChargeStationBlueTraj);
        driveBackwardsOntoChargeStationBlueCommand = driveTrain.generateRamseteCommand(driveBackwardsOntoChargeStationBlueTraj);
        driveForwardOverChargeStationRedCommand = driveTrain.generateRamseteCommand(driveForwardOverChargeStationRedTraj);
        driveBackwardsOntoChargeStationRedCommand = driveTrain.generateRamseteCommand(driveBackwardsOntoChargeStationRedTraj);

        driveForwardOverChargeStationBlueCommand2 = driveTrain.generateRamseteCommand(driveForwardOverChargeStationBlueTraj);
        driveBackwardsOntoChargeStationBlueCommand2 = driveTrain.generateRamseteCommand(driveBackwardsOntoChargeStationBlueTraj);
        driveForwardOverChargeStationRedCommand2 = driveTrain.generateRamseteCommand(driveForwardOverChargeStationRedTraj);
        driveBackwardsOntoChargeStationRedCommand2 = driveTrain.generateRamseteCommand(driveBackwardsOntoChargeStationRedTraj);

        driveBackwardsToCubeBlueCommand = driveTrain.generateRamseteCommand(driveBackwardsToCubeBlueTraj);
        driveForwardsToGridBlueCommand = driveTrain.generateRamseteCommand(driveForwardsToGridBlueTraj);
        driveBackwardsToCubeRedCommand = driveTrain.generateRamseteCommand(driveBackwardsToCubeRedTraj);
        driveForwardsToGridRedCommand = driveTrain.generateRamseteCommand(driveForwardsToGridRedTraj);

        // Initialize autonomous commands here
        balanceChargeStationAuto = Autos.balanceAuto(driveForwardOverChargeStationBlueCommand, driveBackwardsOntoChargeStationBlueCommand,
                driveForwardOverChargeStationRedCommand, driveBackwardsOntoChargeStationRedCommand, balanceCommand, arm);
        placeConeCommandAuto = Autos.placeConeAuto(clawGrip, goTowardsTopRight, goTowardsStartingPos4);
        doublePlacementAuto = Autos.doublePlacementAuto(arm, clawGrip, driveBackwardsToCubeBlueCommand, driveForwardsToGridBlueCommand,
                driveBackwardsToCubeRedCommand, driveForwardsToGridRedCommand, goTowardsTopRight2, goTowardsStartingPos, goTowardsStartingPos2,
                goTowardsStartingPos3, goTowardsPickupPos, goTowardsTopCenter);
        placeConeThenBalanceAuto = Autos.placeConeThenBalanceAuto(driveForwardOverChargeStationBlueCommand2, driveBackwardsOntoChargeStationBlueCommand2, 
                driveForwardOverChargeStationRedCommand2, driveBackwardsOntoChargeStationRedCommand2, balanceCommand2, arm, clawGrip, goTowardsTopRight3, goTowardsStartingPos5);

        // Adds autonomous options to dashboard
        m_chooser.setDefaultOption("Default Auto", defaultAuto);
        m_chooser.addOption("My Auto", customAuto);
        m_chooser.addOption("Balance Charge Station Auto", balanceChargeStationAuto);
        m_chooser.addOption("Place Cone Auto", placeConeCommandAuto);
        m_chooser.addOption("Double Placement Auto", doublePlacementAuto);
        m_chooser.addOption("Place Cone Then Balance Auto", placeConeThenBalanceAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
}

    public void onAutonInit() {
        inTeleop = false;

        m_autonomousCommand = this.getAutonomousCommand();

        System.out.println("Autonomous initiated");
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            System.out.println("Begin autonomous scheduling");
            m_autonomousCommand.schedule();
            System.out.println("Autonomous scheduled");
        }
    }

    public void onTeleopInit() {
        inTeleop = true;
        this.arm.setPIDControlState(false);
        

        this.driveTrain.setDefaultCommand(this.manualDrive);
        this.arm.setDefaultCommand(this.testArmControl);
        //this.arm.setDefaultCommand(this.armControl);
        this.clawGrip.setDefaultCommand(this.clawOpenandCloseCommand);
        this.clawRotation.setDefaultCommand(this.clawRotateCommand);
    }
}
