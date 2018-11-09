package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyRev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.Robot;

@TeleOp(name="SkidSteer_TeleOp_Centering", group="New")
//@Disabled

public class Teleop_Centering extends AbstractTeleop {

    private Robot robot;

    private Rev2mDistanceSensor sensorTimeOfFlight1;
    private DistanceSensor sensorRange1;
    private Rev2mDistanceSensor sensorTimeOfFlight2;
    private DistanceSensor sensorRange2;

    MyRev2mDistanceSensor FirstDistanceSensor;
    MyRev2mDistanceSensor SecondDistanceSensor;

    @Override
    public void Init() {
        robot = new Robot();

        FirstDistanceSensor = new MyRev2mDistanceSensor("distance1");
        SecondDistanceSensor = new MyRev2mDistanceSensor("distance2");



        //sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensor_range1");
        //sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange1;

        //sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensor_range2");
        //sensorTimeOfFlight2 = (Rev2mDistanceSensor)sensorRange2;
    }

    @Override
    public void Loop() {
        //robot.updateDrive();

       // telemetry.addData("deviceName",sensorRange1.getDeviceName() );
        //telemetry.addData("range", String.format("%.01f mm", sensorRange1.getDistance(DistanceUnit.MM)));
        //telemetry.addData("range", String.format("%.01f cm", sensorRange1.getDistance(DistanceUnit.CM)));
        //telemetry.addData("range", String.format("%.01f m", sensorRange1.getDistance(DistanceUnit.METER)));
        //telemetry.addData("range", String.format("%.01f in", FirstDistanceSensor.getDistanceIN() ));

        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight1.getModelID()));
        //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight1.didTimeoutOccur()));

        //telemetry.addData("deviceName",sensorRange1.getDeviceName() );
        //telemetry.addData("range", String.format("%.01f mm", sensorRange2.getDistance(DistanceUnit.MM)));
        //telemetry.addData("range", String.format("%.01f cm", sensorRange2.getDistance(DistanceUnit.CM)));
        //telemetry.addData("range", String.format("%.01f m", sensorRange2.getDistance(DistanceUnit.METER)));
        //telemetry.addData("range", String.format("%.01f in", SecondDistanceSensor.getDistanceIN() ));

        // Rev2mDistanceSensor specific methods.
        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight2.getModelID()));
        //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight2.didTimeoutOccur()));

        //telemetry.update();
    }

    @Override
    public void Stop() {
        robot.stop();
    }

    @Override
    public void a_down()
    {
        robot.centerBack();
    }

    @Override
    public void b_down()
    {
        robot.driveToMyDistanceGyro(0.5, 8000,0, 1);
    }

    @Override
    public void lsy_change(double lsy) {
        robot.setDriveY(lsy);
    }

    @Override
    public void rsx_change(double rsx) {
        robot.setDriveZ(rsx);
    }
}