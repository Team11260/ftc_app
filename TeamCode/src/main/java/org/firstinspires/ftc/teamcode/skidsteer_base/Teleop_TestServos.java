package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.HardwareDevices;

@TeleOp(name="SkidSteer_RevServos", group="Utility")
//@Disabled

public class Teleop_TestServos extends AbstractTeleop {

    private HardwareDevices hardware;

    Servo Servo1;
    Servo Servo2;

    double telemetry_lt_down;
    double telemetry_rt_down;

    @Override
    public void Init()
    {
        hardware = new HardwareDevices();
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        Servo1.setPosition(0);
        Servo2.setPosition(0);
    }

    @Override
    public void Loop() {
        //this function uses the information from the parameters from left_stick_y and right_stick_x
        hardware.updateDrive();
        telemetry.addData("Servo1 Direction = " + Servo1.getDirection());
        telemetry.addData("Servo2 Direction = " + Servo2.getDirection());
        telemetry.addData("tele lt_down = " + telemetry_lt_down);
        telemetry.addData("tele rt_down = " + telemetry_rt_down);
        telemetry.update();

    }

    @Override
    public void Stop() {
        hardware.stop();
    }

    //These two functions below are used to generate values for the updateDrive function
    @Override
    public void lsy_change(double left_stick_y) {
        hardware.setDriveY(left_stick_y);
    }

    @Override
    public void rsx_change(double right_stick_x) {
        hardware.setDriveZ(right_stick_x);
    }

    @Override
    public void a_down()
    {
        Servo1.setPosition(1);
    }

    @Override
    public void a_up()
    {
        Servo1.setPosition(0);
    }

    @Override
    public void b_down()
    {
        Servo2.setPosition(1);
    }

    @Override
    public void b_up()
    {
        Servo2.setPosition(0);
    }

    @Override
    public void x_down()
    {
        if(Servo1.getDirection() == Servo.Direction.FORWARD) {
            Servo1.setDirection(Servo.Direction.REVERSE);
        } else {
            Servo1.setDirection(Servo.Direction.FORWARD);
        }

    }

    @Override
    public void y_down()
    {
        if(Servo2.getDirection() == Servo.Direction.FORWARD) {
            Servo2.setDirection(Servo.Direction.REVERSE);
        } else {
            Servo2.setDirection(Servo.Direction.FORWARD);
        }

    }

    @Override
    public void lt_change(double lt_down)
    {
        Servo1.setPosition(lt_down);
        telemetry_lt_down = lt_down;
    }

    @Override
    public void rt_change(double rt_down)
    {
        Servo2.setPosition(rt_down);
        telemetry_rt_down = rt_down;
    }

}