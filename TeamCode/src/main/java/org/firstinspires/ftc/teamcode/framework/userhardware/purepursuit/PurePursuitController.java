package org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

public abstract class PurePursuitController {

    private final double trackWidth;
    private double lastLeftPosition = 0, lastRightPosition = 0;
    private Pose currentPosition = new Pose();
    private Path currentPath = null;
    private boolean isFollowing = false;
    private boolean isReversed = false;

    public PurePursuitController(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void update() {
        updatePose();

        updateFollower();
    }

    public void updatePose() {

        double leftPosition = getLeftActualPositionInches();
        double rightPosition = getRightActualPositionInches();

        double heading = getActualHeadingDegrees();

        double distance = ((leftPosition - lastLeftPosition) + (rightPosition - lastRightPosition)) / 2;

        currentPosition = new Pose(currentPosition.add(new Vector(distance * Math.cos(Math.toRadians(heading)), distance * Math.sin(Math.toRadians(heading)))), heading);

        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;
    }

    public void updateFollower() {

        if(!isFollowing || currentPath == null) return;

        Pose followPosition = currentPosition.clone();

        if(isReversed) followPosition = new Pose(followPosition.getX(), followPosition.getY(), ((followPosition.getHeading() + 360) % 360) - 180);

        int lookahead = currentPath.getLookAheadPointIndex(followPosition);
        int closest = currentPath.getClosestPointIndex(followPosition);

        if(lookahead == -1) {
            currentPath = null;
            isFollowing = false;
            return;
        }

        double velocity = currentPath.getPathPointVelocity(closest, followPosition);
        double curvature = currentPath.getCurvatureFromPathPoint(lookahead, followPosition);

        double left;
        double right;

        if(isReversed) {
            left = -Range.clip(velocity * ((1.5 - curvature * trackWidth)/1.5), -1, 1);
            right = -Range.clip(velocity * ((1.5 + curvature * trackWidth)/1.5), -1, 1);
        } else {
            left = Range.clip(velocity * ((1.5 + curvature * trackWidth)/1.5), -1, 1);
            right = Range.clip(velocity * ((1.5 - curvature * trackWidth)/1.5), -1, 1);
        }

        AbstractOpMode.getTelemetry().getSmartdashboard().putGraph("Speeds", "L", AbstractOpMode.getTimeSinceStart(), left);
        AbstractOpMode.getTelemetry().getSmartdashboard().putGraph("Speeds", "R", AbstractOpMode.getTimeSinceStart(), right);

        setPowers(left, right);
    }

    public void follow(Path path) {
        currentPath = path;
        isFollowing = true;
    }

    public void setFollowReversed(boolean reversed) {
        isReversed = reversed;
    }

    public boolean getFollowReversed() {
        return isReversed;
    }

    public Pose getCurrentPosition() {
        return currentPosition;
    }

    public boolean isFollowing() {
        return isFollowing;
    }

    public void pause() {
        isFollowing = false;
    }

    public void resume() {
        isFollowing = true;
    }

    public void encodersZero() {
        lastLeftPosition = 0;
        lastRightPosition = 0;
    }

    public void resetPosition() {
        currentPosition = new Pose();
    }

    public abstract double getActualHeadingDegrees();

    public abstract double getLeftActualPositionInches();

    public abstract double getRightActualPositionInches();

    public abstract void setPowers(double l, double r);
}
