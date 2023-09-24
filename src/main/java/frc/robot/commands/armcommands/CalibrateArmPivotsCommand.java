package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.XboxController;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmPivotsCommand extends CommandBase {
    private static final double INIT_SPEED = -0.2;
    private static final double BOOST = -0.2;

    private final ArmSubsystem arm;
    private final NintendoProController controller;

    private static enum CalibrationStates {
        CALIB_PIVOT_2,
        CALIB_PIVOT_1,
        FINISH
    }

    private CalibrationStates calibrationState;

    public CalibrateArmPivotsCommand(ArmSubsystem arm, NintendoProController controller) {
        this.arm = arm;
        this.controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);

        System.out.println("CALIBRATING ARM");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.arm.setPIDControlState(false);
        this.calibrationState = CalibrationStates.CALIB_PIVOT_2;
        this.arm.setPivot1Speed(0);
        this.arm.setPivot2Speed(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("CALIBRATING ARM");
        // this.arm.setPIDControlState(false);
        double speed = INIT_SPEED;
        if (this.controller.getRawButtonWrapper(4)) {
            speed += BOOST;
        }
        switch (this.calibrationState) {
            case CALIB_PIVOT_2:
                if (!this.arm.getPivot2LimitPressed()) {
                    this.arm.setPivot2Speed(speed);
                } else {
                    this.arm.setPivot2Speed(0);
                    this.calibrationState = CalibrationStates.CALIB_PIVOT_1;
                }
                break;
            case CALIB_PIVOT_1:
                if (!this.arm.getPivot1LimitPressed()) {
                    this.arm.setPivot1Speed(speed);
                } else {
                    this.arm.setPivot1Speed(0);
                    this.calibrationState = CalibrationStates.FINISH;
                }
                break;
            case FINISH:
                this.arm.setPivot1Speed(0);
                this.arm.setPivot2Speed(0);
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.arm.setPivot1Speed(0);
        this.arm.setPivot2Speed(0);
        this.arm.resetCoords();
        this.arm.resetArm1Encoder();
        this.arm.resetArm2Encoder();
        this.arm.setPIDControlState(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.calibrationState == CalibrationStates.FINISH || (this.arm.getPivot1LimitPressed() && this.arm.getPivot2LimitPressed());
    }

}
