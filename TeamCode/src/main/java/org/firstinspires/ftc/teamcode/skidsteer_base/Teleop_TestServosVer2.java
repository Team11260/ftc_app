package org.firstinspires.ftc.teamcode.skidsteer_base;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.AbstractTeleop;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.HardwareDevices;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp(name="SkidSteer_RevServosVer2", group="Utility")
//@Disabled

public class Teleop_TestServosVer2 extends AbstractTeleop {

    private HardwareDevices hardware;

    Servo Servo1;
    Servo Servo2;
    CRServo Servo3;

    double telemetry_lt_down;
    double telemetry_rt_down;
    double Servo3Power;

    boolean ServoTestMode;

    @Override
    public void Init()
    {
        hardware = new HardwareDevices();
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        Servo3 = hardwareMap.crservo.get("VexServo");
        Servo1.setPosition(0);
        Servo2.setPosition(0);

        ServoTestMode = false;  // rev = false, vex = true

        Servo3Power = 0.0;
        Servo3.setPower(Servo3Power);
    }

    @Override
    public void Loop() {
        //this function uses the information from the parameters from left_stick_y and right_stick_x
        hardware.updateDrive();
        if ( ServoTestMode  )
        {
            telemetry.addData("SERVO TEST MODE = ", "VEX");
        }
        else
        {
            telemetry.addData("SERVO TEST MODE = ", "REV");
        }

        telemetry.addData(" a = Servo1 0->1, b = Servo2 0->1");
        telemetry.addData(" a = Vex 0.5, b = Vex -0.5, x = Vex 0.0");
        telemetry.addData("Servo1 Direction = " + Servo1.getDirection());
        telemetry.addData("Servo2 Direction = " + Servo2.getDirection());

        telemetry.addData("Servo1 Position = " + Servo1.getPosition());
        telemetry.addData("Servo2 Position = " + Servo2.getPosition());

        telemetry.addData("VerServo Pwr = " + Servo3Power);

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
        if ( !ServoTestMode  )
        {
            Servo1.setPosition(1);
        }
        else {
            Servo3Power = range(round((Servo3Power-0.1),1));
            Servo3.setPower(Servo3Power);
        }
    }

    @Override
    public void a_up()
    {
        Servo1.setPosition(0);
    }

    @Override
    public void b_down()
    {
        if ( !ServoTestMode ) {
            Servo2.setPosition(1);
        }
        else
        {
            Servo3Power = range(round((Servo3Power+0.1),1));
            Servo3.setPower(Servo3Power);
        }
    }

    @Override
    public void b_up()
    {
        Servo2.setPosition(0);
    }

    @Override
    public void x_down()
    {   if ( !ServoTestMode )
        {
             if(Servo1.getDirection() == Servo.Direction.FORWARD) {
                Servo1.setDirection(Servo.Direction.REVERSE);
            } else {
                Servo1.setDirection(Servo.Direction.FORWARD);
            }
        }
        else
        {
            Servo3Power = 0.0;
            Servo3.setPower(Servo3Power);
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

    @Override
    public void start_down()
    {
        if ( ServoTestMode )
        {
            ServoTestMode = false;
        }
        else
        {
            ServoTestMode = true;
        }

    }

    // ********************************
    //   My custom helper methods
    // ********************************
    public static double round(double value, int places) {
        //if (places < 0) throw new IllegalAccessException("places is wrong");
        if (places < 0) {
            places = 0;  // quick and dirty way to handle negative numbers
        }
        BigDecimal bd = new BigDecimal(Double.toString(value));
        // why is there a bd = ?
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    private double range(double val){
        if(val<-1) val=-1;
        if(val>1) val=1;
        return val;
    }


}