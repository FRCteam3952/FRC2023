package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmGyro extends SubsystemBase {

    public static SerialPort arduino;
    public static double gyro_adjust = 0.0;

    static {
        SerialPort.Port[] port_types = new SerialPort.Port[] {
            SerialPort.Port.kUSB,
            SerialPort.Port.kUSB1,
            SerialPort.Port.kUSB2,
        };
        for (SerialPort.Port port_type : port_types) {
            System.out.println(port_type);
            try {
                arduino = new SerialPort(19200, port_type);
                System.out.println("Arm Gyro Connected");
                break;
            } catch (Exception e) {
                System.out.println("Failed to Connect");
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