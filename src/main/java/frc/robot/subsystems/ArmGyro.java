package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmGyro extends SubsystemBase {

    public static SerialPort arduino;
    public static double gyro_adjust = 0.0;

    static {
        try {
            arduino = new SerialPort(19200, SerialPort.Port.kUSB);
            System.out.println("Arm Gyro Connected");
        } catch (Exception e) {
            System.out.println("Failed to Connect, trying kUSB1");
            try {
                arduino = new SerialPort(19200, SerialPort.Port.kUSB1);
                System.out.println("Arm Gyro Connected");
            } catch (Exception e1) {
                System.out.println("Failed to Connect, trying kUSB2");
                try {
                    arduino = new SerialPort(19200, SerialPort.Port.kUSB2);
                    System.out.println("Arm Gyro Connected");
                } catch (Exception e2) {
                    System.out.println("Failed to Connect, all connections failed");
                }
            }
        }
    }

    public static void poke() {
        if (arduino.getBytesReceived() > 0) {
            System.out.println("Arm Gyro Connected");
        } else {
            System.out.println("Arm Gyro not Connected");
        }
    }

    public static Double getGyroAngle() {
        if (arduino.getBytesReceived() > 0) {
            return Double.parseDouble(arduino.readString()) + gyro_adjust;
        } else {
            return 0.0;
        }

    }

    public static void setGyroAngle(double angle) {
        gyro_adjust = -getGyroAngle() + angle;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }
}