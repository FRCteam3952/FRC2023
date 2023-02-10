package frc.robot.subsystems;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ArmConstants;

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

        pivot1Encoder.setPositionConversionFactor(30);
        pivot2Encoder.setPositionConversionFactor(30);
        turretEncoder.setPositionConversionFactor(30);

        this.pidController = new PIDController(0.5, 0, 0); // tune later lol

        this.x_pos = ArmConstants.STARTING_X;
        this.y_pos = ArmConstants.STARTING_Y;
        this.z_pos = ArmConstants.STARTING_Z;
    }

    public double[] getCurrentAngles() {
        double angle1 = pivot1Encoder.getPosition(); // "degrees" - sean
        double angle2 = pivot2Encoder.getPosition();
        double angle3 = turretEncoder.getPosition();
        
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
        if(this.x_pos == x && this.y_pos == y && this.z_pos == z) { // if intended coordinates are same, then don't change target
           return;
        }

        pidController.setTolerance(ArmConstants.ANGLE_DELTA);
        var ikuAngles = InverseKinematicsUtil.getAnglesFromCoordinates(x, y, z);
        var currentAngles = getCurrentAngles();
        setPivot1Speed(pidController.calculate(currentAngles[0], ikuAngles[0]));
        setPivot2Speed(pidController.calculate(currentAngles[1], ikuAngles[1]));
        setTurretSpeed(pidController.calculate(currentAngles[2], ikuAngles[2]));

        // Updates coordinates
        var positions = InverseKinematicsUtil.getCurrentCoordinates();
        this.x_pos = positions[0];
        this.y_pos = positions[1];
        this.z_pos = positions[2];
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