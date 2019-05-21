package org.firstinspires.ftc.teamcode.framework.abstractopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.Logger;
import org.openftc.revextensions2.RevExtensions2;
import org.upacreekrobotics.dashboard.Dashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ConcurrentModificationException;
import java.util.List;

public abstract class AbstractOpMode extends LinearOpMode {

    private List<Exception> exceptions = Collections.synchronizedList(new ArrayList<Exception>());

    //Setup OpMode instance to allow other classes to access hardwareMap and Telemetry
    private static OpMode opmode;
    private static LinearOpMode linearOpMode;
    public static DoubleTelemetry telemetry;
    private static AbstractOpMode thisOpMode;

    public static DoubleTelemetry getTelemetry() {
        return telemetry;
    }

    public static HardwareMap getHardwareMap() {
        return opmode.hardwareMap;
    }

    public static OpMode getOpModeInstance() {
        return opmode;
    }

    public AbstractOpMode() {
        RevExtensions2.init();
        thisOpMode = this;
        opmode = this;
        linearOpMode = this;
        telemetry = new DoubleTelemetry(super.telemetry, Dashboard.getInstance().getTelemetry(), new Logger(Dashboard.getCurrentOpMode()));
    }

    @Override
    public void runOpMode() {
        runOpmode();
    }

    public abstract void runOpmode();

    public static int getTimeSinceInit() {
        return Dashboard.getTimeSinceInit();
    }

    public static int getTimeSinceStart() {
        return Dashboard.getTimeSinceStart();
    }

    public static void delay(int millis) {
        if(Thread.currentThread().isInterrupted()) return;
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static boolean isOpModeActive() {
        return !linearOpMode.isStopRequested();
    }

    protected static void stopRequested() {
        Dashboard.onOpModePreStop();
    }

    public static void staticThrowException(Exception e){
        thisOpMode.throwException(e);
    }

    public void throwException(Exception e) {
        exceptions.add(e);
    }

    public void checkException() {
        for (Exception e : exceptions) {
            telemetry.update();
            for (StackTraceElement element : e.getStackTrace()) {
                if (element.toString().contains("org.firstinspires.ftc.teamcode")) {
                    telemetry.addData(DoubleTelemetry.LogMode.ERROR, element.toString().replace("org.firstinspires.ftc.teamcode.", ""));
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
                case "ExecutionException": {
                    telemetry.update();
                    for (StackTraceElement element : e.getCause().getStackTrace()) {
                        if (element.toString().contains("org.firstinspires.ftc.teamcode")) {
                            telemetry.addData(DoubleTelemetry.LogMode.ERROR, element.toString().replace("org.firstinspires.ftc.teamcode.", ""));
                        }
                    }
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    RuntimeException exception = new RuntimeException();
                    exception.initCause(e);
                    exception.setStackTrace(e.getCause().getStackTrace());
                    throw exception;
                }
                default: {
                    telemetry.update();
                    AbstractOpMode.delay(500);
                    NullPointerException exception = new NullPointerException(e.getMessage());
                    exception.setStackTrace(e.getStackTrace());
                    throw exception;
                }
            }
        }
    }
}
