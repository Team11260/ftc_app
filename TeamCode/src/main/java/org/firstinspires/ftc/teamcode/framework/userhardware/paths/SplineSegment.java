package org.firstinspires.ftc.teamcode.framework.userhardware.paths;

import com.acmerobotics.roadrunner.Pose2d;

public class SplineSegment extends Segment {

    private final Pose2d trajectory;
    private final Pose2d startingPosition;

    public SplineSegment(String name, Pose2d trajectory) {
        this(name, null, trajectory);
    }

    public SplineSegment(String name, Pose2d startingPosition, Pose2d trajectory) {
        super(name, SegmentType.SPLINE);
        this.startingPosition = startingPosition;
        this.trajectory = trajectory;
    }

    public Pose2d getStartingPosition() {
        return startingPosition;
    }

    public Pose2d getTrajectory() {
        return trajectory;
    }
}

