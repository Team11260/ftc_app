package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

@Autonomous(name="SkidSteer_Auto_DriveDistanceTest", group="Utility")
//@Disabled

public class Auton_DriveDistanceTest extends AbstractAuton {


    public ElapsedTime MyAutoTime;

    Robot robot;

    int x,y,finalAngle;

    @Override
    public void Init() {
        MyAutoTime = new ElapsedTime();
        telemetry.addData("Init Hardware");
        telemetry.update();
        robot = new Robot();
        // x = (int)Dashboard.getInputValue("x value");
        //y = (int)Dashboard.getInputValue("y value");
        // finalAngle = (int)Dashboard.getInputValue("final angle");
        telemetry.addData("Ready, Press Play To Start");
        telemetry.update();
    }

    @Override
    public void Run() {

        telemetry.addData("Move to 5,000");
        telemetry.update();
        robot.driveToMyDistance(5000,1);

        telemetry.addData("Move to -5,000");
        telemetry.update();
        robot.driveToMyDistance(-5000,1);


        MyAutoTime.reset();
        while ((MyAutoTime.milliseconds() < 5000)&& isOpModeActive()) {
            //wait for 5 seconds
            // so we can see telemetry
        }

        //  robot.moveTo(x,y,finalAngle);

    }

    @Override
    public void Stop() {
        robot.stop();
    }

}