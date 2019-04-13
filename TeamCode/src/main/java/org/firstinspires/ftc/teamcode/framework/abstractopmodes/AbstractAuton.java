package org.firstinspires.ftc.teamcode.framework.abstractopmodes;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.bogiebase.hardware.RobotState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public abstract class AbstractAuton extends AbstractOpMode {

    private boolean threadRunning = false;

    private List<Exception> exceptions = Collections.synchronizedList(new ArrayList<Exception>());

    public AbstractAuton() {

    }

    @Override
    public void runOpmode() {

        RobotState.currentMatchState = RobotState.MatchState.AUTONOMOUS;

        ExecutorService service = Executors.newSingleThreadExecutor();

        Callable<Boolean> InitThread = () -> {
            try {
                Init();
            } catch (Exception e) {
                throwException(e);
            }
            return true;
        };
        Callable<Boolean> InitLoopThread = () -> {
            try {
                InitLoop();
            } catch (Exception e) {
                throwException(e);
            }
            return true;
        };
        Callable<Boolean> RunThread = () -> {
            try {
                Run();
            } catch (Exception e) {
                throwException(e);
            }
            return true;
        };

        Future<Boolean> CurrentFuture;

        checkException();

        //calls user init
        CurrentFuture = service.submit(InitThread);

        int initLoops = 0;

        while (!isStopRequested() && !isStarted()) {
            checkException();

            if (CurrentFuture.isDone()) {
                initLoops++;
                CurrentFuture = service.submit(InitLoopThread);
            }
        }

        while (!isStopRequested() && !CurrentFuture.isDone()) checkException();

        if (!isStopRequested()) CurrentFuture = service.submit(RunThread);

        while (!isStopRequested() && !CurrentFuture.isDone()) checkException();

        AbstractOpMode.stopRequested();

        //TODO remake our shutdown procedure
        CurrentFuture.cancel(true);

        while (!service.isTerminated()) {
            service.shutdownNow();
            checkException();
        }

        Stop();
        telemetry.stop();
    }

    public abstract void Init();

    public void InitLoop() {

    }

    public abstract void Run();

    public void Stop() {

    }
}