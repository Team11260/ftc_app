package org.firstinspires.ftc.teamcode.rishiandmilan;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractTeleop;
import org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;
import org.firstinspires.ftc.teamcode.framework.util.RobotCallable;
import org.upacreekrobotics.dashboard.Config;

@TeleOp(name = "Learning Teleop Mode", group = "New")

@Config
public class LearningTeleopMode extends AbstractTeleop {

    private static double ARM_DOWN_POSITION = 0.85;
    private static double ARM_UP_POSITION = 0.5;
    private static double GRIPPER_GRIP_POSITION = 0.8;
    private static double GRIPPER_RELEASE_POSITION = 0.5;

    private SlewDcMotor dcMotorFrontLeft;
    private SlewDcMotor dcMotorFrontRight;
    private SlewDcMotor dcMotorBackLeft;
    private SlewDcMotor dcMotorBackRight;

    private Servo arm, gripper;
    boolean up = true, gripped = false;

    public void SetDrivePowerAll(double FL, double FR, double BL, double BR) {
        dcMotorFrontLeft.setPower(FL*0.2);
        dcMotorFrontRight.setPower(FR*0.2);
        dcMotorBackLeft.setPower(BL*0.2);
        dcMotorBackRight.setPower(BR*0.2);


    }

    @Override
    public void RegisterEvents() {
        addEventHandler("1_a_down", () -> toggleArmPosition());
        addEventHandler("1_b_down", () -> toggleGripperPosition());
        addEventHandler("1_x_down", () -> {
            telemetry.addData(DoubleTelemetry.LogMode.INFO, "X pressed");
            telemetry.update();
        });
    }

    @Override
    public void UpdateEvents() {
        double left_stick_x=gamepad1.left_stick_x,left_stick_y = -gamepad1.left_stick_y, right_stick_x = gamepad1.right_stick_x;


        SetDrivePowerAll(left_stick_y-left_stick_x-right_stick_x,left_stick_y+left_stick_x+right_stick_x,left_stick_y+left_stick_x-right_stick_x,left_stick_y-left_stick_x+right_stick_x);
    }

    @Override
    public void Init() throws Exception {
        dcMotorFrontLeft = new SlewDcMotor((hardwareMap.dcMotor.get("front_left")));
        dcMotorFrontRight = new SlewDcMotor((hardwareMap.dcMotor.get("front_right")));
        dcMotorBackLeft = new SlewDcMotor((hardwareMap.dcMotor.get("back_left")));
        dcMotorBackRight = new SlewDcMotor((hardwareMap.dcMotor.get("back_right")));

        dcMotorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        dcMotorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        dcMotorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        dcMotorBackRight.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.servo.get("arm_servo");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(ARM_UP_POSITION);
        gripper = hardwareMap.servo.get("gripper_servo");
        gripper.setDirection(Servo.Direction.FORWARD);
        gripper.setPosition(GRIPPER_RELEASE_POSITION);
    }

    public void setGripperPosition(double position) {
        gripper.setPosition(position);
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
    }

    public void toggleArmPosition() {
        setArmPosition(up ? ARM_DOWN_POSITION : ARM_UP_POSITION);
        up = !up;
    }

    public void toggleGripperPosition() {
        setGripperPosition(gripped ? GRIPPER_GRIP_POSITION : GRIPPER_RELEASE_POSITION);
        gripped = !gripped;
    }

    public void setArmUp() {
        arm.setPosition(ARM_UP_POSITION);
    }

    public void setArmDown() {
        arm.setPosition(ARM_DOWN_POSITION);
    }

    public void setGripperGrip() {
        gripper.setPosition(GRIPPER_GRIP_POSITION);
    }

    public void setGripperRelease() {
        gripper.setPosition(GRIPPER_RELEASE_POSITION);
    }


    public RobotCallable setGripperGripCallable() {
        return () -> setGripperGrip();
    }


    public RobotCallable setGripperReleaseCallable() {
        return () -> setGripperRelease();
    }

    @Override
    public void Loop() throws Exception {

    }

    @Override
    public void Stop() {

    }
}
