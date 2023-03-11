package frc.robot.commands.armcommands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateArmTurretCommand extends CommandBase {
    private final ArmSubsystem arm;
    private double initialAngle = 0;
    private double endAngle = 0;
    private static final double kp = 0.01;
    private static final double ki = 0.0;
    private static final double kd = 0.0;
    private static final PIDController turretPID = new PIDController(kp, ki, kd);

    private static final double MAX_SPEED = 0.5;

    public CalibrateArmTurretCommand(ArmSubsystem arm, double endAngle) {
        this.arm = arm;
        this.endAngle = endAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
    public CalibrateArmTurretCommand(ArmSubsystem arm) {
        this.arm = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.arm.setTurretSpeed(0);
        this.initialAngle = this.arm.getTurretAngleDeg();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turretSpeed = turretPID.calculate(this.arm.getTurretAngleDeg(),endAngle);
        arm.setTurretSpeed(MathUtil.clamp(turretSpeed, -MAX_SPEED, MAX_SPEED));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.arm.setTurretSpeed(0);
        this.arm.resetTurretEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(arm.getTurretAngleDeg() - endAngle) < 10);
    }

}
