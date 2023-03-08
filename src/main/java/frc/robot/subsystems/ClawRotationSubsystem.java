// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.staticsubsystems.LimeLight;


public class ClawRotationSubsystem extends SubsystemBase {
    private final CANSparkMax clawRotator;
    private final RelativeEncoder clawRotationEncoder;

    private final PIDController clawPidController;

    private double targetAngle;

    public ClawRotationSubsystem() {
        this.clawRotator = new CANSparkMax(PortConstants.CLAW_ROTATE_PORT, MotorType.kBrushless);
        this.clawRotationEncoder = this.clawRotator.getEncoder();
        this.clawRotationEncoder.setPositionConversionFactor(30); // each motor rotation is 30 degrees

        this.clawPidController = new PIDController(1e-2, 0, 0);
        this.clawPidController.setTolerance(ClawConstants.CORRECT_CLAW_ROTATION_AT_DELTA);
        this.targetAngle = 0;
    }
    public void changeAngle(double changeBy){
        targetAngle += changeBy;
    }

    public void setAngle(double angle) {
        double clawSpeed = clawPidController.calculate(getClawAngle(),angle);
        clawSpeed = Math.min(ClawConstants.ROTATE_MAX_OUTPUT, Math.max(clawSpeed, ClawConstants.ROTATE_MIN_OUTPUT));
        this.setClawRotateSpeed(clawSpeed);
    }

    public void setClawRotateSpeed(double speed) {
        this.clawRotator.set(speed);
    }

    // Automatically rotates claw to match angle when designated button is held down
    public CommandBase autoRotate() {
        return this.runOnce(() -> {
            clawRotator.set(LimeLight.getAngleAdjustment());
        });
    }

    public double getClawAngle() {
        return this.clawRotationEncoder.getPosition();
    }

    @Override
    public void periodic() {
        setAngle(targetAngle);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
