// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ClawRotationSubsystem extends SubsystemBase {
    private final CANSparkMax clawRotator;

    private final RelativeEncoder clawRotationEncoder;

    public ClawRotationSubsystem() {
        this.clawRotator = new CANSparkMax(PortConstants.CLAW_ROTATE_PORT, MotorType.kBrushless);

        this.clawRotationEncoder = this.clawRotator.getEncoder();
    }

    public void setAngle(double angle){
        double difference = this.getClawAngle() - angle;
        if(Math.abs(difference) < ClawConstants.ANGLE_DELTA) {
            this.setClawRotateSpeed(0);
        } else if(difference > 0) {
            this.setClawRotateSpeed(ClawConstants.CLAW_ROTATE_SPEED);
        } else {
            this.setClawRotateSpeed(-ClawConstants.CLAW_ROTATE_SPEED);
        }
    }

    public void setClawRotateSpeed(double speed) {
        this.clawRotator.set(speed);
    }

    public double getClawAngle(){
        return this.clawRotationEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
