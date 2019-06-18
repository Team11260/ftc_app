package org.firstinspires.ftc.teamcode.rishiandmilan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractAuton;
import org.upacreekrobotics.dashboard.Config;

@Autonomous(name = "RishiMilanOpMode", group = "Example")
//@Disabled

@Config
public class RishiMilanOpMode extends AbstractAuton {
        private DcMotor dcMotorLeft;
        private DcMotor dcMotorRight;


    @Override
    public void RegisterStates() {

    }

    @Override
    public void Init() {
        dcMotorLeft=hardwareMap.dcMotor.get("left");
        dcMotorRight=hardwareMap.dcMotor.get("right");
        dcMotorRight.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void Run() {
        dcMotorLeft.setPower(1);
        dcMotorRight.setPower(1);
        delay(2000);
        dcMotorLeft.setPower(0);
        dcMotorRight.setPower(0);


    }


}
