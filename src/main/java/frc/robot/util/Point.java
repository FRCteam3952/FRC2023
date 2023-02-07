package frc.robot.util;

class Point {
    /* A point in 3D space relative to the robot's center.
     * - X is right/left, I think positive is right.
     * - Y is up/down, positive is up.
     * - Z is forward/backward, positive is forward.
     * 
     * All measurements are in inches.
    */
    public double x;
    public double y;
    public double z;

    // make the point
    public Point (double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // immutable add method
    public Point add(Point p) {
        return new Point(this.x + p.x, this.y + p.y, this.z + p.z);
    }

    // maybe we need it as a list for backwards compatibility?
    public double[] as_list() {
        return new double[] {this.x, this.y, this.z};
    }
}