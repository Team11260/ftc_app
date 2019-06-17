package org.firstinspires.ftc.teamcode.rishiandmilan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAuton;
import org.upacreekrobotics.dashboard.Config;

@Autonomous(name = "RishiMilanOpMode", group = "Example")
//@Disabled

@Config
public class RishiMilanOpMode extends AbstractAuton {

    public static int time = 3000;
    public static double speed1 = 0.3;
    public static double speed2 = 0.6;

    private DcMotor dcMotorLeft;
    private DcMotor dcMotorRight;

    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {

        dcMotorLeft = hardwareMap.dcMotor.get("left");
        dcMotorRight = hardwareMap.dcMotor.get("right");

        dcMotorRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void Run() {

        GoForward(speed1);
        GoForward(speed2);

    }

    public void GoForward(double mspeed){
        dcMotorLeft.setPower(mspeed);
        dcMotorRight.setPower(mspeed);
        delay(time);
        dcMotorLeft.setPower(0);
        dcMotorRight.setPower(0);
    }
}
