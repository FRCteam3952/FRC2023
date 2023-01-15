package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * self explanatory
 */

public class Gyro extends SubsystemBase {
  private static ADIS16470_IMU gyro;
  private static double angleAdjust = 0;

  public Gyro() {
    gyro = new ADIS16470_IMU();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
    gyro.calibrate();
  }

  public static double getGyroAngle() {
    return gyro.getAngle() + angleAdjust;
  }

  public static void setGyroAxis(ADIS16470_IMU.IMUAxis axis) {
    gyro.setYawAxis(axis);
  }

  public static void resetGyroAngle() {
    gyro.reset();
    angleAdjust = 0;
  }
  public static void setGyroAngle(double angle){
    resetGyroAngle();
    angleAdjust = angle;
  }

  @Override
  public void periodic() {
    //System.out.println(getGyroAngle());
    // resetGyroAngle();
  }

  @Override
  public void simulationPeriodic() {

  }
}