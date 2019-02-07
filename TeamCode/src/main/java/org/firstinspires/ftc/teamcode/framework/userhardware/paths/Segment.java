package org.firstinspires.ftc.teamcode.framework.userhardware.paths;

public abstract class Segment {

    private boolean isRunning = false, isDone = false;

    private int number;

    private final String name;

    private final SegmentType type;

    public Segment(String name, SegmentType type) {
        this.name = name;
        this.type = type;
    }

    protected synchronized void setNumber(int number) {
        this.number = number;
    }

    protected synchronized int getNumber() {
        return number;
    }

    public synchronized String getName() {
        return name;
    }

    public synchronized SegmentType getType() {
        return type;
    }

    protected synchronized void reset() {
        isRunning = false;
        isDone = false;
    }

    protected synchronized void start() {
        isRunning = true;
        isDone = false;
    }

    protected synchronized void stop() {
        isRunning = false;
        isDone = true;
    }

    protected synchronized void pause() {
        isRunning = false;
    }

    protected synchronized void resume() {
        isRunning = true;
    }

    public synchronized boolean isRunning() {
        return isRunning;
    }

    public synchronized boolean isDone() {
        return isDone;
    }

    public enum SegmentType {
        DRIVE,
        TURN
    }
}

