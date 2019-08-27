package org.firstinspires.ftc.teamcode.mecanum_base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.userhardware.inputs.sensors.vision.vuforia.RelicVuMarkTracker;
import org.firstinspires.ftc.teamcode.mecanum_base.hardware.Robot;

@Autonomous(name="Mecanum_Auto_New", group="New")
//@Disabled

public class Auton extends AbstractAuton {

    Robot robot;
    RelicVuMarkTracker vuforia;

    @Override
    public void Init() {
        robot = new Robot();
        vuforia = new RelicVuMarkTracker();
    }

    @Override
    public void Run() {
        while(opModeIsActive()) {
            telemetry.addData(vuforia.getVuMark().name());
            double[] pose = vuforia.getPose();
            if(pose != null){
                telemetry.addData("x: "+pose[0]+"y: "+pose[1]+"z: "+pose[2]);
                robot.setDriveX(pose[0]/200.00);
                robot.setDriveY(-((500+pose[2])/200.00));
                robot.driveUpdate();
            }
            else {
                robot.setDriveX(0);
                robot.setDriveY(0);
                robot.driveUpdate();
            }
            telemetry.update();
        }
    }

    @Override
    public void Stop() {

    }
}