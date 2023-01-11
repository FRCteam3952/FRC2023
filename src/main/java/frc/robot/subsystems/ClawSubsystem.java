// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PortConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.ClawCommands.ClawGripCommand;
import frc.robot.commands.ClawCommands.ClawRotateCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax clawGrip;
    private final CANSparkMax clawRotation;

    private final RelativeEncoder clawGripEncoder;
    private final RelativeEncoder clawRotationEncoder;

    private double intendedAngle;
    private boolean clawState;


    public ClawSubsystem() {
        this.clawGrip = new CANSparkMax(PortConstants.PIVOT1_PORT, MotorType.kBrushless);
        this.clawRotation = new CANSparkMax(PortConstants.PIVOT2_PORT, MotorType.kBrushless);

        this.clawGripEncoder = this.clawGrip.getEncoder();
        this.clawRotationEncoder = this.clawRotation.getEncoder();

        this.intendedAngle = 0;
        this.clawState = false;
    }

    public void setAngle(double angle){
        this.intendedAngle = angle;
        ClawRotateCommand useClaw = new ClawRotateCommand(this, this.intendedAngle);
        useClaw.schedule();

    }

    public void openClaw(){
        this.clawState = true;
        ClawGripCommand useClaw = new ClawGripCommand(this, true);
        useClaw.schedule();

    }

    public void closeClaw(){
        this.clawState = false;
        ClawGripCommand useClaw = new ClawGripCommand(this, false);
        useClaw.schedule();

    }

    public boolean getClawState(){
        return this.clawState;
    }

    public double getClawAngle(){
        return this.clawRotationEncoder.getPosition();
    }

    public double getClawGripEncoder(){
        return this.clawGripEncoder.getPosition();
    }

    public void setClawRotateSpeed(double speed){
        clawRotation.set(speed);
    }
    public void setClawGripSpeed(double speed){
        clawGrip.set(speed);
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
