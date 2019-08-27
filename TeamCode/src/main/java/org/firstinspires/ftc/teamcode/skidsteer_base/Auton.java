package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.Pixycam;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

@Autonomous(name="SkidSteer_Auto_733pm", group="AAA")
//@Disabled

public class Auton extends AbstractAuton {


    public ElapsedTime MyAutoTime;

    Robot robot;

    Pixycam pixy;

    @Override
    public void Init() {
        pixy = hardwareMap.get(Pixycam.class,"pixy");
        robot = new Robot();
    }

    @Override
    public void Run() {
        while (true) {
            try {
                Pixycam.Block largestBlock = pixy.getLargestBlock();
                if (largestBlock != null) {
                    do {
                        largestBlock = pixy.getLargestBlock();
                        double x = (largestBlock.getX() - 150) / 150.0;
                        if (x > 1) x = 1;
                        if (x < -1) x = -1;
                        robot.setDriveY(0.6);
                        robot.setDriveZ(x);
                        robot.updateDrive();
                        telemetry.addDataPhone("X: " + x);
                        telemetry.update();
                    } while((largestBlock!=null)&& isOpModeActive());
                } else {
                    robot.setDriveY(0);
                    robot.setDriveZ(-0.5);
                    robot.updateDrive();
                    robot.delay(200);
                    robot.setDriveY(0);
                    robot.setDriveZ(0);
                    robot.updateDrive();
                    largestBlock = pixy.getLargestBlock();
                    if (largestBlock != null) continue;
                    robot.setDriveY(0);
                    robot.setDriveZ(0.5);
                    robot.updateDrive();
                    robot.delay(400);
                    robot.setDriveY(0);
                    robot.setDriveZ(0);
                    robot.updateDrive();
                }
            } catch (NullPointerException e){

            }
        }
    }

    @Override
    public void Stop() {
        robot.stop();
    }
}