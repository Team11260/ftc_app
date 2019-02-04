package org.firstinspires.ftc.teamcode.framework.userhardware.paths;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAutonNew;

import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;

public class Path implements Cloneable {

    private ConcurrentHashMap<Integer, Segment> segments = new ConcurrentHashMap<>();
    private Segment currentSegment = null;

    private int numSegments = 0;

    private boolean paused = false;

    private boolean isDone = false;

    private final String name;

    public Path(String name) {
        this.name = name;
    }

    public void reset() {
        for(HashMap.Entry<Integer, Segment> segment:segments.entrySet()){
            segment.getValue().reset();
        }
        currentSegment = null;
        paused = false;
        isDone = false;
    }

    public void addSegment(Segment segment) {
        segment.setNumber(numSegments);
        segments.put(numSegments, segment);
        numSegments++;
    }

    public Segment getNextSegment() {
        if (currentSegment == null) {
            currentSegment = segments.get(0);
            currentSegment.start();
            return currentSegment;
        } else {
            currentSegment.stop();
        }

        String lastSegmentName = currentSegment.getName();

        if (currentSegment.getNumber() >= segments.size() - 1) {
            isDone = true;
            AbstractAutonNew.addFinishedState(lastSegmentName);
            return null;
        }

        currentSegment = segments.get(currentSegment.getNumber() + 1);

        currentSegment.start();

        if (paused) currentSegment.pause();

        AbstractAutonNew.addFinishedState(lastSegmentName);

        return currentSegment;
    }

    public Segment getCurrentSegment() {
        if (currentSegment == null) return segments.get(0);
        return currentSegment;
    }

    public void nextSegment() {
        currentSegment.stop();
    }

    public void pause() {
        paused = true;
        currentSegment.pause();
    }

    public void resume() {
        paused = false;
        currentSegment.resume();
    }

    public boolean isPaused() {
        return paused;
    }

    public boolean isDone() {
        return isDone;
    }

    public String getName() {
        return name;
    }
}
