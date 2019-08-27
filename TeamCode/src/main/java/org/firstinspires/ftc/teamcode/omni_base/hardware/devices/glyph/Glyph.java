package org.firstinspires.ftc.teamcode.omni_base.hardware.devices.glyph;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.SlewDcMotor;

public class Glyph {
    Servo clawservo;
    SlewDcMotor clawmotor;
    SlewDcMotor lift;

    public Glyph(HardwareMap hwMap) {
        clawservo = hwMap.servo.get("clawservoT");
        clawservo.setPosition(0.4);
        clawservo.setDirection(Servo.Direction.REVERSE);
        clawmotor = new SlewDcMotor(hwMap.dcMotor.get("glyphclaw"));
        lift = new SlewDcMotor(hwMap.dcMotor.get("gripper"));

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0);

        clawmotor.setDirection(DcMotor.Direction.REVERSE);
        clawmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawmotor.setPower(0);
    }

    public void setClawServoPosition(double position) {
        clawservo.setPosition(position);
    }

    public void setClawPower(double power){
        clawmotor.setPower(power);
    }

    public void setLiftPosition(int position, double power){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(position);
        lift.setPower(power);
    }

    public enum liftHeights{

        BOTTOM(0),
        LOW(750),
        MEDIUM(1050),
        HIGH(1400);

        private int height;

        private liftHeights(int height){
            this.height = height;
        }

        public int getHeight(){
            return height;
        }

        public static liftHeights up(liftHeights value){
            switch (value){
                case BOTTOM:
                    return LOW;
                case LOW:
                    return MEDIUM;
                case MEDIUM:
                    return HIGH;
                case HIGH:
                    return HIGH;
            }
            return BOTTOM;
        }

        public static liftHeights down(liftHeights value){
            switch (value){
                case BOTTOM:
                    return BOTTOM;
                case LOW:
                    return BOTTOM;
                case MEDIUM:
                    return LOW;
                case HIGH:
                    return MEDIUM;
            }
            return BOTTOM;
        }
    }
}
