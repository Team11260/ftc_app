package org.firstinspires.ftc.teamcode.framework.userhardware.paths;

import com.acmerobotics.roadrunner.Pose2d;

public class SplineSegment extends Segment{

    private Pose2d finalLocation;

    public SplineSegment(String name, Pose2d finalLocation) {
        super(name, SegmentType.SPLINE);
        this.finalLocation = finalLocation;
    }

    public Pose2d getFinalLocation() {
        return finalLocation;
    }
}
