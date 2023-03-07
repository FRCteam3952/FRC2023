package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class ArmGyro {

    public static SerialPort arduino;
    public static double gyro_adjust = 0.0;
    public static double saved_angle = 0.0;

    static {
        SerialPort.Port[] port_types = new SerialPort.Port[]{SerialPort.Port.kUSB, SerialPort.Port.kUSB1, SerialPort.Port.kUSB2};
        for (SerialPort.Port port_type : port_types) {
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
        if (arduino != null && arduino.getBytesReceived() > 0) {
            System.out.println("Arm Gyro Connected");
        } else {
            System.out.println("Arm Gyro not Connected");
        }
    }

    public static double getGyroAngle() {
        if (arduino != null && arduino.getBytesReceived() > 0) {
            try {
                String[] read = arduino.readString().split("\n");
                if (read.length > 0){
                    String angle = read[read.length-1];
                    double val = Double.parseDouble(angle) - gyro_adjust;
                    saved_angle = val;
                    return val;
                }
                else{
                    return saved_angle;
                }

            } catch (Exception e) {
                return saved_angle;
            }
        } else {
            return saved_angle;
        }

    }

    public static void setGyroAngle(double angle) {
        gyro_adjust = saved_angle;
    }
}