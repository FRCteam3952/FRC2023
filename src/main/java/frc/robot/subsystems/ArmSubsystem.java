package frc.robot.subsystems;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ArmInverseKinematicsConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.InverseKinematicsUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax pivot1;
    private final CANSparkMax pivot2;
    private final CANSparkMax turret;

    private final RelativeEncoder pivot1Encoder;
    private final RelativeEncoder pivot2Encoder;
    private final RelativeEncoder turretEncoder;

    private final PIDController pidController;

    private double x_pos;
    private double y_pos;
    private double z_pos;
    
    //arm control constructor
    public ArmSubsystem() {
        this.pivot1 = new CANSparkMax(PortConstants.PIVOT1_PORT, MotorType.kBrushless);
        this.pivot2 = new CANSparkMax(PortConstants.PIVOT2_PORT, MotorType.kBrushless);
        this.turret = new CANSparkMax(PortConstants.TURRET_PORT, MotorType.kBrushless);

        this.pivot1Encoder = this.pivot1.getEncoder();
        this.pivot2Encoder = this.pivot2.getEncoder();
        this.turretEncoder = this.turret.getEncoder();

        this.pidController = new PIDController(0.5, 0, 0); //tune later lol

        this.x_pos = ArmInverseKinematicsConstants.STARTING_X;
        this.y_pos = ArmInverseKinematicsConstants.STARTING_Y;
        this.z_pos = ArmInverseKinematicsConstants.STARTING_Z;
    }

    public double[] getCurrentAngles() {
        double angle1 = pivot1Encoder.getPosition() * 1 + 0; // sean are these radians or degrees pls help
        double angle2 = pivot2Encoder.getPosition() * 1 + 0;
        double angle3 = turretEncoder.getPosition() * 1 + 0;
        
        return new double[] {angle1,angle2,angle3};
    }
    public void setPivot1Speed(double speed) {
        this.pivot1.set(speed);
    }

    public void setPivot2Speed(double speed) {
        this.pivot2.set(speed);
    }
    
    public void setTurretSpeed(double speed) {
        this.turret.set(speed);
    }

    public void stopAllMotors() {
        this.pivot1.set(0);
        this.pivot2.set(0);
        this.turret.set(0);
    }
    
    public double[] getCurrentCoordinates(){
        return new double[]{this.x_pos,this.y_pos,this.z_pos};
    }

    public void setIntendedCoordinates(double x, double y, double z){
        if(this.x_pos == x && this.y_pos == y && this.z_pos == z) //if intended coordinates are same, then don't change target
           return; 

        pidController.setTolerance(ArmInverseKinematicsConstants.ANGLE_DELTA);
        setPivot1Speed(pidController.calculate(x_pos, InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z)[0]));
        setPivot2Speed(pidController.calculate(y_pos, InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z)[1]));
        setTurretSpeed(pidController.calculate(z_pos, InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z)[2]));

        this.x_pos = x;
        this.y_pos = y;
        this.z_pos = z;

        
        
        // MoveArmToAngleCommand adjustArm = new MoveArmToAngleCommand(this, InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z));
        // adjustArm.schedule();
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