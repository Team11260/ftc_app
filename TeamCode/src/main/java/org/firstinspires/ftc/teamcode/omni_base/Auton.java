package org.firstinspires.ftc.teamcode.omni_base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.omni_base.hardware.Robot;

@Autonomous(name="Omni_Auto_New", group="New")
//@Disabled

public class Auton extends AbstractAuton {

    protected Robot robot;

    @Override
    public void Init() {
        robot = new Robot();
    }

    @Override
    public void Run() {
        robot.delay(2000);
        robot.speak("I will lower my jewel arm");
        telemetry.addData("Lowering");
        telemetry.update();
        while (robot.isSpeaking());
        robot.jewelArmDown();

        robot.delay(2000);
        robot.speak("I will raise my jewel arm");
        telemetry.addData("Raising");
        telemetry.update();
        while (robot.isSpeaking());
        robot.jewelArmUp();

        robot.delay(2000);
        robot.speak("I will close my claw");
        telemetry.addData("Closing");
        telemetry.update();
        while (robot.isSpeaking());
        robot.glyphClawClose();

        robot.speak("I will lift my arm");
        telemetry.addData("Lifting");
        telemetry.update();
        while (robot.isSpeaking());
        robot.glyphLiftUp();
        robot.glyphLiftUp();
        robot.glyphLiftUp();
        robot.glyphLiftUp();

        robot.delay(2000);
        robot.speak("I will lower my arm");
        telemetry.addData("Lowering");
        telemetry.update();
        while (robot.isSpeaking());
        robot.glyphLiftDown();
        robot.glyphLiftDown();
        robot.glyphLiftDown();
        robot.glyphLiftDown();

        robot.delay(2000);
        robot.speak("I will open my claw");
        telemetry.addData("Opening");
        telemetry.update();
        while (robot.isSpeaking());
        robot.glyphClawOpen();

        robot.delay(2000);
        robot.speak("Done");
        telemetry.addData("Done");
        telemetry.update();
    }

    @Override
    public void Stop() {

    }
}