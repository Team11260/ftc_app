package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public abstract class AbstractTeleop extends AbstractOpMode {

    private List<Exception> exceptions = Collections.synchronizedList(new ArrayList<Exception>());

    //Setup gamepad
    private Emitter emitter = new Emitter();
    private ElapsedTime emitTime;
    private long emitTimeOffset = 0;
    private int emitLoop = 0;

    ButtonStateMap states;
    FloatStateMap floatStates;

    public AbstractTeleop(){

    }

    @Override
    public void runOpMode() {

        ExecutorService service = Executors.newSingleThreadExecutor();

        Callable<Boolean> InitThread = ()->{
            try {
                Init();
            } catch (Exception e){ throwException(e); }
            return true;
        };
        Callable<Boolean> InitLoopThread = ()->{
            try {
                InitLoop();
            } catch (Exception e){ throwException(e); }
        return true;
        };
        Callable<Boolean> StartThread = ()-> {
        try{
            Start();
        } catch (Exception e){ throwException(e); }
            return true;
        };
        Callable<Boolean> LoopThread = ()->{
            try {
                Loop();
            } catch (Exception e){ throwException(e); }
        return true;
        };

        Future<Boolean> CurrentFuture;

        //sets up emitter
        emitTime = new ElapsedTime();
        states = new ButtonStateMap();
        floatStates = new FloatStateMap();

        //calls user init
        CurrentFuture = service.submit(InitThread);

        telemetry.addData("Init");
        telemetry.update();

        int initLoops = 0;

        telemetry.addData("Init Loop");
        telemetry.update();

        while (!isStopRequested() && !isStarted()){
            checkException();

            if(CurrentFuture.isDone()) {
                initLoops++;
                CurrentFuture = service.submit(InitLoopThread);
            }
        }

        telemetry.addData(initLoops+" Init Loops");
        telemetry.update();

        while (!isStopRequested() && !CurrentFuture.isDone());

        telemetry.addData("Starting");
        telemetry.update();

        RegisterEvents();

        if(!isStopRequested()) {
            checkException();

            CurrentFuture = service.submit(StartThread);
        }

        emitTime.reset();

        while (opModeIsActive()) {
            checkException();

            //checks the gamepad for changes
            checkEvents();

            //calls user loop
            if(CurrentFuture.isDone()) {
                CurrentFuture = service.submit(LoopThread);
            }
        }

        telemetry.addData("Stopping");
        telemetry.update();

        AbstractOpMode.stopRequested();

        //TODO remake our shutdown procedure
        CurrentFuture.cancel(true);
        emitter.shutdown();

        while (!service.isTerminated()) {
            service.shutdownNow();
            checkException();
        }

        Stop();
    }

    public abstract void RegisterEvents();

    public abstract void UpdateEvents();

    public abstract void Init();

    public void InitLoop(){

    }

    public void Start(){

    }

    public abstract void Loop();

    public void Stop(){

    }

    private void throwException(Exception e){
        exceptions.add(e);
    }

    private void checkException() {
        for (Exception e : exceptions) {
            telemetry.update();
            for (StackTraceElement element : e.getStackTrace()) {
                if (element.toString().contains("org.firstinspires.ftc.teamcode")) {
                    telemetry.addData(element.toString().replace("org.firstinspires.ftc.teamcode.", ""));
                }
            }
            switch (e.getClass().getSimpleName()) {
                case "NullPointerException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    NullPointerException exception = (NullPointerException) e;
                    throw exception;
                }
                case "IllegalArgumentException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    IllegalArgumentException exception = (IllegalArgumentException) e;
                    throw exception;
                }
                case "ArrayIndexOutOfBoundsException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    ArrayIndexOutOfBoundsException exception = (ArrayIndexOutOfBoundsException) e;
                    throw exception;
                }
                case "ConcurrentModificationException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    ConcurrentModificationException exception = (ConcurrentModificationException) e;
                    throw exception;
                }
                case "IllegalStateException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    IllegalStateException exception = (IllegalStateException) e;
                    throw exception;
                }
                case "VuforiaException": {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    VuforiaException exception = (VuforiaException) e;
                    throw exception;
                }
                default: {
                    telemetry.addData(e.getClass().getSimpleName());
                    telemetry.update();
                    AbstractOpMode.delay(2000);
                }
            }
        }
    }

    public void addEventHandler(String name, Callable event){
        emitter.on(name, event);
    }

    public void pauseEvent(String name){
        emitter.pauseEvent(name);
    }

    public void resumeEvent(String name){
        emitter.resumeEvent(name);
    }

    private void checkEvents() {
        if(emitTime.milliseconds()-emitTimeOffset<1)return;
        emitTimeOffset++;
        if(emitTimeOffset>60000){
            emitTimeOffset = 0;
            emitTime.reset();
        }
        //emitter.refresh(telemetry);

        // boolean buttons
        checkBooleanInput("a", gamepad1.a);
        checkBooleanInput("b", gamepad1.b);
        checkBooleanInput("x", gamepad1.x);
        checkBooleanInput("y", gamepad1.y);
        checkBooleanInput("lb", gamepad1.left_bumper);
        checkBooleanInput("rb", gamepad1.right_bumper);
        checkBooleanInput("dpl", gamepad1.dpad_left);
        checkBooleanInput("dpr", gamepad1.dpad_right);
        checkBooleanInput("dpu", gamepad1.dpad_up);
        checkBooleanInput("dpd", gamepad1.dpad_down);
        checkBooleanInput("start", gamepad1.start);
        checkBooleanInput("back", gamepad1.back);
        checkBooleanInput("lsb", gamepad1.left_stick_button);
        checkBooleanInput("rsb", gamepad1.right_stick_button);

        // float buttons
        checkFloatInput("lt", gamepad1.left_trigger);
        checkFloatInput("rt", gamepad1.right_trigger);
        checkFloatInput("lsx", gamepad1.left_stick_x);
        checkFloatInput("lsy", gamepad1.left_stick_y);
        checkFloatInput("rsx", gamepad1.right_stick_x);
        checkFloatInput("rsy", gamepad1.right_stick_y);

        //Check user events
        UpdateEvents();

        //Update value for repeat on checkBooleanInput
        emitLoop++;
        if(emitLoop>10000){
            emitLoop=0;
        }
    }

    public void checkBooleanInput(String name, boolean val) {
        if(name.contains("down")){
            if(val){
                emitter.emit(name);
            }
        } else if(name.contains("up")){
            if(!val){
                emitter.emit(name);
            }
        } else if (states.isChanged(name, val)) {
            emitter.emit(val ? name+"_down" : name+"_up");
            states.change(name, val);
        }
    }

    public void checkBooleanInput(String name, boolean val, int repeat) {
        if(emitLoop%repeat==0){
            checkBooleanInput(name, val);
        }
    }

    public void checkFloatInput(String name, float val) {
        if (floatStates.isChanged(name, val)) {
            emitter.emit(name + "_change");
            floatStates.change(name, val);
        }
    }

    // This is a lazy map. It only tracks buttons that have been pressed. It can
    // easily scale to handle as many buttons as you have unique names.
    class ButtonStateMap {

        ConcurrentHashMap<String, Boolean> state;

        ButtonStateMap() {
            state = new ConcurrentHashMap<String,Boolean>();
        }

        boolean isChanged(String name, boolean newVal) {
            if (!state.containsKey(name)) {
                AbstractOpMode.getTelemetry().addData("Adding: "+name);
                AbstractOpMode.getTelemetry().update();
                state.put(name, newVal);
                return newVal;
            }

            return state.get(name) != newVal;
        }

        void change(String name, boolean newVal) {
            state.replace(name, newVal);
        }
    }

    // This is a float variant of a ButtonStateMap.
    class FloatStateMap {

        ConcurrentHashMap<String, Float> state;

        FloatStateMap() {
            state = new ConcurrentHashMap<String,Float>();
        }

        boolean isChanged(String name, float newVal) {
            if (!state.containsKey(name)) {
                // If it's not 0, it's changed
                state.put(name, newVal);
                return newVal == 0.0;
            }

            return state.get(name) != newVal;
        }

        void change(String name, float newVal) {
            state.replace(name, newVal);
        }
    }
}