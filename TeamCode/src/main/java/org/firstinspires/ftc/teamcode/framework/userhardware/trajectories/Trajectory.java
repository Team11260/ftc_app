package org.firstinspires.ftc.teamcode.framework.userhardware.trajectories;

public abstract class Trajectory {

    public double initialHeading;

    public double finalHeading;

    public abstract double distance(double distance);

    public abstract double velocity(double distance);

    public abstract double acceleration(double distance);

    public abstract double heading(double distance);

    public abstract double curvature(double distance);

    public abstract boolean isDone(double time);

    public double getInitialAndFinalHeadingDelta() {
        return this.finalHeading - this.initialHeading;
    }
}