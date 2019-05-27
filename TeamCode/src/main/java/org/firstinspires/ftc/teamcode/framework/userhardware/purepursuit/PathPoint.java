package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

public class PathPoint extends Point {

    private final double distance, curvature;
    private double velocity;

    public PathPoint(double x, double y, double distance, double curvature, double velocity) {
        super(x, y);
        this.distance = distance;
        this.curvature = curvature;
        this.velocity = velocity;
    }

    public PathPoint(Point point, double distance, double curvature, double velocity) {
        this(point.getX(), point.getY(), distance, curvature, velocity);
    }

    public double getDistance() {
        return distance;
    }

    public double getCurvature() {
        return curvature;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }
}
