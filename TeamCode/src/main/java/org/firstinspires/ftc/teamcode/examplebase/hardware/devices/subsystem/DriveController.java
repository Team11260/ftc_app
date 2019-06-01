package org.firstinspires.ftc.teamcode.examplebase.hardware.devices.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.util.SubsystemController;
import org.upacreekrobotics.dashboard.Dashboard;

public class DriveController extends SubsystemController {

    private Drive drive;

    //Utility Methods
    public DriveController() {
        init();
    }

    public synchronized void init() {
        drive = new Drive(hardwareMap);
    }

    public synchronized void update() {
        telemetry.getSmartdashboard().putGraph("Encoders", "Left", Dashboard.getTimeSinceStart(), drive.getLeftCurrentPosition());
        telemetry.getSmartdashboard().putGraph("Encoders", "Right", Dashboard.getTimeSinceStart(), drive.getRightCurrentPosition());
    }

    public void driveInches(int inches){
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPower(0.4,0.4);
        while((drive.getLeftCurrentPosition()+drive.getRightCurrentPosition())<(76*inches)){
            telemetry.addData(DoubleTelemetry.LogMode.INFO,drive.getLeftCurrentPosition()+drive.getRightCurrentPosition() );
        }
        drive.setPower(0,0);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnLeft(int angle){
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPower(-0.3,0.3);
        while (drive.getHeading()<angle);
        drive.setPower(0,0);
        drive.resetHeading();
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRight(int angle){
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPower(0.3,-0.3);
        while (drive.getHeading()>(-angle));
        drive.setPower(0,0);
        drive.resetHeading();
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public synchronized void stop() {
        drive.stop();
    }
}