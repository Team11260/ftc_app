package org.firstinspires.ftc.teamcode.framework.util;

import java.util.concurrent.Callable;

public class PathState extends State {

    @Deprecated
    public PathState(String name, String previousState, Callable<Boolean> run) {
        super(name, previousState, run);
    }

    public PathState(String name, String previousState, RobotCallable run) {
        super(name, previousState, run);
    }
}
