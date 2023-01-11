package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private double origin_height; //all measurement in inches?
    private final double LIMB1_LENGTH;
    private final double LIMB2_LENGTH;
    private double turretAngle, a1, a2;

    //arm control constructor
    public Arm(double height,double l1,double l2) {
        this.origin_height = height;
        this.LIMB1_LENGTH = l1;
        this.LIMB2_LENGTH = l2;
    }
    public double distance(double x1, double x2, double y1, double y2) { //2d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
    }
    public double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2) + Math.pow(z1-z2,2));
    }
    public double lawOfCosines(double a, double b, double c) { //law of cosines calc
        return Math.toDegrees(Math.acos((Math.pow(a,2) + Math.pow(b,2) - Math.pow(c,2))/(2*(a*b))));
    }
    public double lawOfSines(double angle, double a, double b) { //law of sines calc
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }
    public double angleBetweenLines(double x1, double y1, double x2, double y2) { //angle between lines
        double dotproduct = x1 * x2 + y1 * y2;
        return Math.toDegrees(Math.acos(dotproduct/(Math.abs(distance(0,x1,0,y1)*distance(0, x2, 0, y2)))));
    }
    //calculate arm angles relevative to limb that it's attatched to
    public void calcAngles(double x, double y, double z) {
        double relative_y = y - origin_height;                          // calc distance in 3d from top pivot point
        double adjusted_x = Math.abs(x);
        double dist3d = distance(0,x,0,relative_y,0,z);
        if(dist3d > LIMB1_LENGTH + LIMB2_LENGTH){                       //checks if arm coordinate is possible
            return;
        }
        double dist2d = distance(0,adjusted_x,0,relative_y);                                         // calc distance in 2d from top pivot point
        a2 = lawOfCosines(LIMB1_LENGTH, LIMB2_LENGTH, dist2d);                                              // a2 is angle between 1st arm segment to 2nd arm segment
        a1 = angleBetweenLines(0, -1, adjusted_x, relative_y) - lawOfSines(a2, dist2d, LIMB2_LENGTH);   // a1 is angle between verticle to 1st arm segment

        //ill optimze later cuz lol
        if (x == 0){
            if(z > 0){
                turretAngle = 90;
            }
            else{
                turretAngle = 270;
            }
        }
        else if(x > 0){
            if(z > 0){
                turretAngle = Math.toDegrees(Math.atan(Math.abs(z/x)));
            }
            else{
                turretAngle = 360 - Math.toDegrees(Math.atan(Math.abs(z/x)));
            }
        }
        else{
            if(z > 0){
                turretAngle = 180 - Math.toDegrees(Math.atan(Math.abs(z/x)));
            }
            else{
                turretAngle = 180 + Math.toDegrees(Math.atan(Math.abs(z/x)));
            }
        }
    }
    public double getTurretAngle() {
        return turretAngle;
    }
    public double getFirstAngle() {
        return a1;
    }
    public double getSecondAngle() {
        return a2;
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