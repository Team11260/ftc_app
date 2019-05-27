package org.firstinspires.ftc.teamcode.framework.util;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public class State {

    private final String name, previousState;
    private final Callable<Boolean> run;
    private Future<Boolean> future = null;

    @Deprecated
    public State(String name, String previousState, Callable<Boolean> run) {
        this.name = name;
        this.previousState = previousState;
        this.run = run;
    }

    public State(String name, String previousState, RobotCallable run) {
        this(name, previousState, () -> {
            run.call();
            return true;
        });
    }

    public String getName() {
        return name;
    }

    public Callable<Boolean> getRun() {
        return run;
    }

    public String getPreviousState() {
        return previousState;
    }

    public void setFuture(Future<Boolean> future) {
        this.future = future;
    }

    public Object getValue() {
        if(future == null || !future.isDone()) return null;
        try {
            return future.get();
        } catch (InterruptedException e) {
        } catch (ExecutionException e) {
            AbstractOpMode.staticThrowException(e);
        }
        return null;
    }

    public boolean isDone() {
        if (future != null) return future.isDone();
        return true;
    }

    public void cancel() {
        if(future != null) future.cancel(true);
    }
}