package org.firstinspires.ftc.teamcode.mecanum_base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.AbstractAuton;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;
import org.upacreekrobotics.dashboard.Dashboard;

@Autonomous(name="Mecanum_Test_CreatePath_New", group="New")
//@Disabled

public class CreatePath extends AbstractAuton{

    int length = 1000;

    int[] frontleft = new int[length];
    int[] frontright = new int[length];
    int[] backleft = new int[length];
    int[] backright = new int[length];

    SlewDcMotor[] motors = new SlewDcMotor[4];

    @Override
    public void Init() {
        motors[0] = new SlewDcMotor(hardwareMap.dcMotor.get("front_left"));
        motors[1] = new SlewDcMotor(hardwareMap.dcMotor.get("front_right"));
        motors[2] = new SlewDcMotor(hardwareMap.dcMotor.get("back_left"));
        motors[3] = new SlewDcMotor(hardwareMap.dcMotor.get("back_right"));

        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.REVERSE);

        for (SlewDcMotor motor: motors) {
            motor.setSlewSpeed(2);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Can be float if required
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }

    @Override
    public void Run() {
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime totaltime = new ElapsedTime();
        runtime.reset();
        totaltime.reset();

        telemetry.addData("Start: "+totaltime.milliseconds());
        telemetry.update();

        for (int i = 0; i < length; i++) {
            while (runtime.milliseconds() < 5);
            frontleft[i] = motors[0].getCurrentPosition();
            frontright[i] = motors[1].getCurrentPosition();
            backleft[i] = motors[2].getCurrentPosition();
            backright[i] = motors[3].getCurrentPosition();
            runtime.reset();
        }

        printArrays();

        telemetry.addData("Path Created: "+totaltime.milliseconds());
        telemetry.update();

        Dashboard.getInputValue("Press Enter To Continue");

        telemetry.addData("Run Path: "+totaltime.milliseconds());
        telemetry.update();

        for (SlewDcMotor motor: motors) {
            motor.setSlewSpeed(2);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Can be float if required
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }

        runtime.reset();

        for (int i = 0; i < 1000; i++) {
            while (runtime.milliseconds() < 10) ;
            motors[0].setTargetPosition(frontleft[i]);
            motors[1].setTargetPosition(frontright[i]);
            motors[2].setTargetPosition(backleft[i]);
            motors[3].setTargetPosition(backright[i]);
            runtime.reset();
        }

        for (SlewDcMotor motor: motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Stop: "+totaltime.milliseconds());
        telemetry.update();
    }

    public void printArrays(){
        String message = "";
        message = message + "int[] frontleft = {";
        for (int i = 0; i < length-1; i++) {
            message = message + frontleft[i] + ",";
        }
        message = message + frontleft[length-1] + "};";
        telemetry.addData(message);

        message = "";
        message = message+"int[] frontright = {";
        for(int i=0;i<length-1;i++){
            message = message+frontright[i]+",";
        }
        message = message+frontright[length-1]+"};";
        telemetry.addData(message);

        message = "";
        message = message+"int[] backleft = {";
        for(int i=0;i<length-1;i++){
            message = message+backleft[i]+",";
        }
        message = message+backleft[length-1]+"};";
        telemetry.addData(message);

        message = "";
        message = message+"int[] backright = {";
        for(int i=0;i<length-1;i++){
            message = message+backright[i]+",";
        }
        message = message+backright[length-1]+"};";
        telemetry.addData(message);
    }
}