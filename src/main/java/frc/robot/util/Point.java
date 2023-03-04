package frc.robot.util;

import java.util.function.BiFunction;

public class Point {
    // immutable point with x, y, and z
    private double x, y, z;

    public Point(double x, double y, double z) 
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Point(double[] positions)
    {
        this.x = positions[0];
        this.y = positions[1];
        this.z = positions[2];
    }
    public Point()
    {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Point fromAngles(double x_angle, double y_angle, double length) 
    { // TODO
        throw new Error(); // todo
    }
    public Point fromXZY(double x, double z, double y) { return new Point(x, y, z); }
    public Point fromXZYList(double[] x) { 
        return fromXZY(x[0], x[1], x[2]); 
    }
    public Point fromMeters(double x, double y, double z)
    {
        return new Point(
            x * 39.3701,
            y * 39.3701,
            z * 39.3701
        );
    }
    public Point fromMetersList(double[] x) {
        return fromMeters(x[0], x[1], x[2]);
    }

    public double distanceFromCenter() { 
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2) + Math.pow(this.z, 2));
    }

    
    public Point combineFields(Point other, BiFunction<Double, Double, Double> combine_func)
    {
        return new Point(
            combine_func.apply(this.x, other.getX()),
            combine_func.apply(this.y, other.getY()),
            combine_func.apply(this.z, other.getZ())
        );
    }


    public double[] getCoordinates() {
        return new double[]{
            this.x,
            this.y,
            this.z,
        };
    }

    public double getX() { return this.x; }
    public double getY() { return this.y; }
    public double getZ() { return this.z; }

    public Point add(Point other) { return combineFields( other, (a, b) -> a + b );          }
    public Point sub(Point other) { return combineFields( other, (a, b) -> a - b );          }
    public Point mul(Point other) { return combineFields( other, (a, b) -> a * b );          }
    public Point div(Point other) { return combineFields( other, (a, b) -> a / b );          }
    public Point mod(Point other) { return combineFields( other, (a, b) -> a % b );          }
    public Point pow(Point other) { return combineFields( other, (a, b) -> Math.pow(a,b) );  }
    

}
