package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

public class Pose extends Point {

    private final double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        this(point.getX(), point.getY(), heading);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public double getHeading() {
        return heading;
    }
}
