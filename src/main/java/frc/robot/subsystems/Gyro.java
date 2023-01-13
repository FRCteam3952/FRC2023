package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * self explanatory
 */

public class Gyro extends SubsystemBase {
  private static ADIS16470_IMU gyro;

  public Gyro() {
    gyro = new ADIS16470_IMU();
    gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
    gyro.calibrate();
  }

  public static double getGyroAngle() {
    return gyro.getAngle();
  }

  public static void setGyroAxis(ADIS16470_IMU.IMUAxis axis) {
    gyro.setYawAxis(axis);
  }

  public static void resetGyroAngle() {
    gyro.reset();
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