package org.firstinspires.ftc.teamcode.mecanum_base.hardware.devices.drive;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.framework.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.SubsystemController;
import org.firstinspires.ftc.teamcode.framework.userhardware.MyNumberRound;
import org.firstinspires.ftc.teamcode.framework.userhardware.outputs.Logger;
import org.firstinspires.ftc.teamcode.framework.userhardware.purepursuit.Path;
import org.firstinspires.ftc.teamcode.skidsteer_base.hardware.MyRev2mDistanceSensor;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import org.firstinspires.ftc.teamcode.framework.RobotParametersList;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.delay;
import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.isOpModeActive;


public class DriveController extends SubsystemController {

    private Drive drive;
    private MyNumberRound DriveControllerRound;
    private MyRev2mDistanceSensor dist;
    private MyRev2mDistanceSensor dist1;
    //these are for the joysticks
    private double turn_x = 0, turn_y = 0, turn_z = 0, fl = 0, fr = 0, bl = 0, br = 0, Drive_Power = 0.1;

    //For testing each motor
    private double Button_Power = 1;
    private double aButton_pressed = 0, bButton_pressed = 0, xButton_pressed = 0, yButton_pressed = 0;
    boolean myTestDriveButtonInProgress = false;

    final File path =
            Environment.getExternalStoragePublicDirectory
                    (
                            //Environment.DIRECTORY_PICTURES
                            Environment.DIRECTORY_DCIM
                    );

    final File file = new File(path, "mylog.txt");

    FileOutputStream fOut;
    OutputStreamWriter myOutWriter;

    Logger logger;
    ElapsedTime runtime;
    DecimalFormat DFenc,DFpwr;

    RobotParametersList paramEncoder;


    public DriveController() {
        init();

    }

    public void init() {
        opModeSetup();

        logger = new Logger("mecanumMotor.txt");
        dist = new MyRev2mDistanceSensor("dist");
        dist1 = new MyRev2mDistanceSensor("dist1");
//        dist = null;
//        dist1 = null;
        runtime = new ElapsedTime();

        DriveControllerRound = new MyNumberRound();

        //Put general setup here
        drive = new Drive(hwMap);
        //This opens up the file to write data in
        try {
            if (file.exists()) {
                RobotLog.i("ABCD Existing File 758pm");
                // do not create file
            } else {
                RobotLog.i("ABCD Creating New File");
                file.createNewFile();
            }
            fOut = new FileOutputStream(file);
            myOutWriter = new OutputStreamWriter(fOut);

        } catch (IOException e) {
            Log.e("Exception", "File init failed " + e.toString());
        }

        try {
            if (file.exists()) {
                RobotLog.i("ABCD Existing File 758pm");
                // do not create file
            } else {
                RobotLog.i("ABCD Creating New File");
                file.createNewFile();
            }
            fOut = new FileOutputStream(file);
            myOutWriter = new OutputStreamWriter(fOut);

        } catch (IOException e) {
            Log.e("Exception", "File init failed " + e.toString());
        }

        /*switch(Mode) {
            case Auton:
                // Put auton setup here
                drive.setSlewSpeed(1);
                break;
            case Teleop:
                // Put teleop setup here
                drive.setSlewSpeed(0.1);
                break;
        }*/
        drive.setSlewSpeed(1);  // temp fix for mecanun stable driver code
        //DFpwr = new DecimalFormat(" #.000;-#.000");
        DFpwr = new DecimalFormat(" #.000");
        DFpwr.setMinimumIntegerDigits(1);
        DFenc = new DecimalFormat(" 00000.0;-00000.0");
       // DFenc.setMinimumIntegerDigits(5);
        paramEncoder = new RobotParametersList();
    }

    public void setX(double x) {
        turn_x = x;
        turn_x = (float) scaleInput(turn_x);
    }

    public void setY(double y) {
        turn_y = y;
        turn_y = (float) scaleInput(turn_y);
    }

    public void setZ(double z) {
        turn_z = z;
        turn_z = (float) scaleInput(turn_z);
    }


    public void setNewPower(double Left, double Right) {
        drive.setNewPower(Left, Right);
    }

    public void runPath(Path path) {
        drive.follow(path);

        while (drive.isFollowing() && isOpModeActive()) {
            AbstractOpMode.telemetry.addData("X" + drive.getCurrentPosition().getX() + "       Front Left = "+ getFrontLeftPosition() +"  Back Left = "+ getBackLeftPosition());
            AbstractOpMode.telemetry.addData("Y" + drive.getCurrentPosition().getY() + "       Front Right ="+ getFrontRightPosition() +"  Back Right = "+ getBackRightPosition());
            drive.update();
        }
        setPower(0, 0, 0, 0);
    }

    public void measureEncoders(int MotorSelected, double milliSecondsStop) {

        // 0 == frontleft
        // 1 == frontright
        // 2 == Backright
        // 3 == Backleft
        double FrontLeftValue1;
        double FrontRightValue1;
        double BackLeftValue1;
        double BackRightValue1;
        double FrontLeftValue2;
        double FrontRightValue2;
        double BackLeftValue2;
        double BackRightValue2;
        double FrontLeftValue3;
        double FrontRightValue3;
        double BackLeftValue3;
        double BackRightValue3;


        logger.log("ABCD BEGIN  measureEncoders");
        //if the joystick is being used don't run the code below
        if ((fl != 0) || (fr != 0) || (bl != 0) || (br != 0)) {
            return;
        }
        myTestDriveButtonInProgress = true;
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //After setting encoder set to 0
        RobotLog.i("measureEncoders Front Left  = " + drive.getFrontLeftPosition());
        RobotLog.i("measureEncoders Front Right = " + drive.getFrontRightPosition());
        RobotLog.i("measureEncoders Back Left  = " + drive.getBackLeftPosition());
        RobotLog.i("measureEncoders Back Right = " + drive.getBackRightPosition());

        logger.log("Start Left Front = " + drive.getFrontLeftPosition());
        logger.log("Start Right Front= " + drive.getFrontRightPosition());
        logger.log("Start Left Back = " + drive.getFrontLeftPosition());
        logger.log("Start Right Back= " + drive.getFrontRightPosition());

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (MotorSelected == 0) {
            logger.log(" MEASURING FrontLeft MOTOR");
        } else if (MotorSelected == 1) {
            logger.log(" MEASURING FrontRIGHT MOTOR");
        } else if (MotorSelected == 2) {
            logger.log(" MEASURING BackLeft MOTOR");
        } else if (MotorSelected == 3) {
            logger.log(" MEASURING BackRIGHT MOTOR");
        }


        runtime.reset();
        if (MotorSelected == 0) {
            drive.setPower(range(1), range(0), range(0), range(0));
        } else if (MotorSelected == 1) {
            drive.setPower(range(0), range(1), range(0), range(0));
        } else if (MotorSelected == 2) {
            drive.setPower(range(0), range(0), range(1), range(0));
        } else if (MotorSelected == 3) {
            drive.setPower(range(0), range(0), range(0), range(1));
        }

        while (runtime.milliseconds() <= milliSecondsStop) {
            // allow motors to run until time is reached
        }

        drive.setPower(range(0), range(0), range(0), range(0));

        FrontLeftValue1 = drive.getFrontLeftPosition();
        FrontRightValue1 = drive.getFrontRightPosition();
        BackLeftValue1 = drive.getBackLeftPosition();
        BackRightValue1 = drive.getBackRightPosition();


        delay(600);
        FrontLeftValue2 = drive.getFrontLeftPosition();
        FrontRightValue2 = drive.getFrontRightPosition();
        BackLeftValue2 = drive.getBackLeftPosition();
        BackRightValue2 = drive.getBackRightPosition();

        delay(600);
        FrontLeftValue3 = drive.getFrontLeftPosition();
        FrontRightValue3 = drive.getFrontRightPosition();
        BackLeftValue3 = drive.getBackLeftPosition();
        BackRightValue3 = drive.getBackRightPosition();

        logger.log("end Front left measure  1 = " + FrontLeftValue1);
        logger.log("end Front Right measure 1 = " + FrontRightValue1);
        logger.log("end Back left measure   1 = " + BackLeftValue1);
        logger.log("end Back Right measure  1 = " + BackRightValue1);


        logger.log("end Front left measure  2 = " + FrontLeftValue2);
        logger.log("end Front Right measure 2 = " + FrontRightValue2);
        logger.log("end Back left measure   2 = " + BackLeftValue2);
        logger.log("end Back Right measure  2 = " + BackRightValue2);

        logger.log("end Front left measure  3 = " + FrontLeftValue3);
        logger.log("end Front Right measure 3 = " + FrontRightValue3);
        logger.log("end Back left measure   3 = " + BackLeftValue3);
        logger.log("end Back Rightmeasure   3 = " + BackRightValue3);


        myTestDriveButtonInProgress = false;
        logger.log("END measureEncoders \n");


    }

    public void resetAllEncoders() {
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delay(100);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getFrontLeftPosition() {
        return drive.getFrontLeftPosition();
    }

    public double getFrontRightPosition() {
        return drive.getFrontRightPosition();
    }

    public double getBackLeftPosition() {
        return drive.getBackLeftPosition();
    }

    public double getBackRightPosition() {
        return drive.getBackRightPosition();
    }

    public void strafeToMyDistance(double distance, double speed) {

        //if the joystick is being used don't run the code below
        if ((fl != 0) || (fr != 0) || (bl != 0) || (br != 0)) {
            return;
        }
        myTestDriveButtonInProgress = true;
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //After setting encoder set to 0
        RobotLog.i("ABCD Front Left Motor Position after reset encoders  = " + drive.getFrontLeftPosition()
                + " " + drive.getFrontRightPosition());
        RobotLog.i("ABCD Back Right Motor Position after reset encoders = " + drive.getBackLeftPosition()
                + " " + drive.getBackRightPosition());

        saveToMyLog("ABCD Front Left Motor Position after reset encoders  1 = " + drive.getFrontRightPosition()
                + " " + drive.getFrontLeftPosition());
        saveToMyLog("ABCD Back Right Motor Position after reset encoders 1 = " + drive.getBackRightPosition()
                + " " + drive.getBackLeftPosition());

        while (
                isOpModeActive() && (
                        (drive.getFrontLeftPosition() != 0) || (drive.getFrontRightPosition() != 0) ||
                                (drive.getBackLeftPosition() != 0) || (drive.getBackRightPosition() != 0))
                ) {
            // Sometimes, it will get stuck, so try to reset again
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addData("Front Left = ", drive.getFrontLeftPosition());
//            telemetry.addData("Front Right = ", drive.getFrontRightPosition());
//            telemetry.addData("Back Left = ", drive.getBackLeftPosition());
//            telemetry.addData("Back Right = ", drive.getBackRightPosition());

            // wait till it equals zero
            // Both motors must be 0 to exit out of the while
        }

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance < 0) {
            speed = -1 * speed;
        }


        drive.setPower(range(speed), range(-speed), range(-speed), range(speed));
//        telemetry.addData("Drive set power", distance);
//        telemetry.update();
        while (((abs(drive.getBackLeftPosition()) < abs(distance)) &&
                (abs(drive.getFrontLeftPosition()) < abs(distance)) &&
                (abs(drive.getFrontRightPosition()) < abs(distance)) &&
                (abs(drive.getBackRightPosition()) < abs(distance))) && isOpModeActive()) {
            //wait here and do nothing until motor reaches the distance
        }

        drive.setPower(range(0), range(0), range(0), range(0));

//        telemetry.addData("Front Left Position ", drive.getFrontLeftPosition());
//        telemetry.addData("Front Right Position ", drive.getFrontRightPosition());
//        telemetry.addData("Back Left Position ", drive.getBackLeftPosition());
//        telemetry.addData("Back Right Position ", drive.getBackRightPosition());
//        telemetry.update();
        RobotLog.i("ABCD Front Left Motor Position  = " + drive.getFrontLeftPosition());
        RobotLog.i("ABCD Front Right Motor Position = " + drive.getFrontRightPosition());
        RobotLog.i("ABCD Back Left Motor Position  = " + drive.getBackLeftPosition());
        RobotLog.i("ABCD Back Right Motor Position = " + drive.getBackRightPosition());


        //waitMyTimer(5000);
        myTestDriveButtonInProgress = false;

    }

    public void strafeToMyDistanceGyro(double speed, double distance, double target_angle, double p_multiplier, int direction) {
        //if the direction is equal to 0, the robot is going sideways.
        //if the direction is equal to 1, the robot is going forwards or backwards.

        double finalBackLeftTarget;
        double finalBackRightTarget;
        double finalFrontLeftTarget;
        double finalFrontRightTarget;
        double error;

        saveToMyLog(" ");

        saveToMyLog("speed = " + DriveControllerRound.roundDouble(speed, 2) +
                " distance = " + DriveControllerRound.roundDouble(distance, 2) +
                " target_angle = " + DriveControllerRound.roundDouble(target_angle, 2) +
                " p_multiplier = " + DriveControllerRound.roundDouble(p_multiplier, 2) +
                " direction = " + DriveControllerRound.roundDouble(direction, 2)
        );

        if ((fl != 0) || (fr != 0) || (bl != 0) || (br != 0)) {
            return;
        }
        myTestDriveButtonInProgress = true;

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //This resets the encoder counts
        while (((drive.getFrontLeftPosition() != 0) || (drive.getFrontRightPosition() != 0) ||
                (drive.getBackLeftPosition() != 0) || (drive.getBackRightPosition() != 0)) && isOpModeActive()) {
            // Sometimes, it will get stuck, so try to reset againh
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addData("Front Left = ", drive.getFrontLeftPosition());
//            telemetry.addData("Front Right = ", drive.getFrontRightPosition());
//            telemetry.addData("Back Left = ", drive.getBackLeftPosition());
//            telemetry.addData("Back Right = ", drive.getBackRightPosition());
//            telemetry.update();
            // / wait till it equals zero
            // Both motors must be 0 to exit out of the while
        }

        //drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //if the user is requesting to backwards or a negative distance, then make the speed negative
        if (distance < 0) {
            speed = -1 * speed;
        }

        finalBackRightTarget = drive.getBackRightPosition() + distance;
        finalBackLeftTarget = drive.getBackLeftPosition() + distance;
        finalFrontRightTarget = drive.getFrontRightPosition() + distance;
        finalFrontLeftTarget = drive.getFrontLeftPosition() + distance;
        // starting  telemetry
//        telemetry.addData("driveTo " + speed + " distance ", distance);
        telemetry.update();
        if (direction == 0) {
            drive.setPower(range(speed), range(-speed), range(-speed), range(speed));
        }
        if (direction == 1) {
            drive.setPower(range(speed), range(speed), range(speed), range(speed));
        }

        double currentAngle;
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        while (
                isOpModeActive() &&
                        (abs(drive.getFrontRightPosition()) < abs(finalFrontRightTarget)) &&
                        (abs(drive.getFrontLeftPosition()) < abs(finalFrontLeftTarget)) &&
                        (abs(drive.getBackLeftPosition()) < abs(finalBackLeftTarget)) &&
                        (abs(drive.getBackRightPosition()) < abs(finalBackRightTarget))
                ) {
            //If you are turning left, it is a positive angle
            //If you are turning Right, it is a negative angle

            //sample 1   = -20 - 0 = -20
            //sample 2   = 20 - 0 = 20
            currentAngle = getHeading();
            error = (target_angle - currentAngle) / 100;
            error = error * p_multiplier;
            // formula is for robot moving forward
            //                  Left Motor                    Right Motor
            //        Sample 1  .5 + (-20) = -19.5        .5 - (-20) = 20.5
            //        Sample 2  .5 + (+20) = 20.5         .5 - (+20) = -19.5
            //if the direction is equal to 0, the robot is going sideways.
            //if the direction is equal to 1, the robot is going forwards or backwards.
            if (direction == 0) {
                leftFrontPower = range(speed - error);
                rightFrontPower = -range(speed - error);
                leftBackPower = -range(speed + error);
                rightBackPower = range(speed + error);
            }
            if (direction == 1) {
                leftFrontPower = range(speed - error);
                leftBackPower = range(speed - error);
                rightFrontPower = range(speed + error);
                rightBackPower = range(speed + error);
            }

            drive.setPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            saveToMyLog("Error = " + DriveControllerRound.roundDouble(error, 2) +
                    " Angle1 = " + DriveControllerRound.roundDouble(currentAngle, 2) +
                    " LFP = " + DriveControllerRound.roundDouble(leftFrontPower, 2) +
                    " RFP = " + DriveControllerRound.roundDouble(rightFrontPower, 2) +
                    " LBP = " + DriveControllerRound.roundDouble(leftBackPower, 2) +
                    " RBP = " + DriveControllerRound.roundDouble(rightBackPower, 2) +
                    "     " +
                    " P1 = " + DriveControllerRound.roundDouble(drive.getFrontLeftPosition(), 2) +
                    " P2 = " + DriveControllerRound.roundDouble(drive.getFrontRightPosition(), 2) +
                    " P3 = " + DriveControllerRound.roundDouble(drive.getBackLeftPosition(), 2) +
                    " P4 = " + DriveControllerRound.roundDouble(drive.getBackRightPosition(), 2)
            );
        }

        drive.setPower(0, 0, 0, 0);
//        telemetry.addData("FL " + drive.getFrontLeftPosition() + " FR " + drive.getFrontRightPosition()
//                , "BL " + drive.getBackLeftPosition() + " BR " + drive.getBackRightPosition()
//        );

        telemetry.update();
        myTestDriveButtonInProgress = true;

    }

    public void saveToMyLog(String FileLingString) {
        String timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());

        try {
            myOutWriter.append(timeStamp + " : " + FileLingString + (char) Character.LINE_SEPARATOR);
            myOutWriter.flush();
        } catch (IOException e) {
            Log.e("Exception", "File append failed: " + e.toString());
        }
    }


    public void encoderStraight(double speed, double distance, double multiplier)
    {
        double LeftPower = speed;
        double RightPower = speed;
        double BackLeftCount;
        double BackRightCount;
        double difference_raw;
        double error;
        int    loop_samples = 0;
        boolean boolContinueloop = true;
        double speedFromFile;

        speedFromFile = paramEncoder.getParamValueDouble("SPEED","0.4");
        if ( speed >= 0 ){  speed = speedFromFile;
        }
        else {              speed = -1.0 * speedFromFile;
        }

        telemetry.addData(" ----------- Speed = " + speed);

        runtime.reset();

        setNewPower(LeftPower, RightPower);
        while ( isOpModeActive() &&
                (   (Math.abs(getFrontLeftPosition()) < distance )
                 && (Math.abs(getFrontRightPosition()) < distance )
                ) && ( boolContinueloop == true)
              )
        {
            loop_samples++;
            BackLeftCount = getBackLeftPosition();
            BackRightCount = getBackRightPosition();

            error = Math.abs(BackLeftCount) - Math.abs(BackRightCount);
            difference_raw = error;
            error = (error / 1000) * multiplier;
            if (speed > 0) {
                if (error < 0) {
                    LeftPower = range(speed + Math.abs(error));
                    RightPower = range(speed - Math.abs(error));
                } else {
                    LeftPower = range(speed - Math.abs(error));
                    RightPower = range(speed + Math.abs(error));
                }
            }
            else{
                    if (error > 0) {
                        LeftPower = range(speed + Math.abs(error));
                        RightPower = range(speed - Math.abs(error));
                    } else {
                        LeftPower = range(speed - Math.abs(error));
                        RightPower = range(speed + Math.abs(error));
                    }
            }

            // Error out of the loop for these conditions
            if (  (( Math.abs(LeftPower) > 0.77 ) && (Math.abs(RightPower) < 0.25 ))
                 || (( Math.abs(LeftPower) < 0.25 ) && (Math.abs(RightPower) > 0.75 ))     )
            {
                telemetry.addData(" EARLY STOP =============== ");
                telemetry.addData(" OD Encoders:  left  =" + DFenc.format(Math.abs(BackLeftCount)) + "  right = " + DFenc.format(Math.abs(BackRightCount))
                        + "  diff = " + DFenc.format(difference_raw)
                        + "     Error " + DFpwr.format(error)
                        + "  Left Power = " + DFpwr.format(LeftPower) + "   Right Power = " + DFpwr.format(RightPower)
                );
                telemetry.update();
                break;
            }

            setNewPower(LeftPower, RightPower);
            telemetry.addData(" OD Encoders:  left  =" + DFenc.format(Math.abs(BackLeftCount)) + "  right = " + DFenc.format(Math.abs(BackRightCount))
                            + "  diff = " + DFenc.format(difference_raw)
                             + "     Error " + DFpwr.format(error)
                             + "  Left Power = " + DFpwr.format(LeftPower) + "   Right Power = " + DFpwr.format(RightPower)
                             );
            telemetry.update();
        } // while loop

        setNewPower(0, 0);
        telemetry.addData("Elapsed Time  =" + runtime.milliseconds());
        telemetry.addData("Front Left Encoders  =" + getFrontLeftPosition());
        telemetry.addData("Front Right Encoders =" + getFrontRightPosition());
        telemetry.addData("Back Left Encoders  =" + getBackLeftPosition());
        telemetry.addData("Back Right Encoders =" + getBackRightPosition());
        telemetry.addData("Loop Samples = ", loop_samples);

        telemetry.addData("Now turning to Adjust");
        telemetry.update();
        delay(5000);
        if (speed < 0) {
            if (Math.abs(getBackLeftPosition()) >= Math.abs(getBackRightPosition())) {
                while (isOpModeActive() && Math.abs(getBackLeftPosition()) >= Math.abs(getBackRightPosition())) {
                    setNewPower(0.2, -0.2);
                    telemetry.addData("Back Left Encoders =" + Math.abs(getBackLeftPosition()));
                    telemetry.addData("Back Right Encoders =" + Math.abs(getBackRightPosition()));
                    telemetry.update();
                }

            } else {
                while (isOpModeActive() && Math.abs(getBackRightPosition()) >= Math.abs(getBackLeftPosition())) {
                    setNewPower(-0.2, 0.2);
                    telemetry.addData("Back Left Encoders =" + Math.abs(getBackLeftPosition()));
                    telemetry.addData("Back Right Encoders =" + Math.abs(getBackRightPosition()));
                    telemetry.update();
                }
            }
        } else {
            if (Math.abs(getBackLeftPosition()) >= Math.abs(getBackRightPosition())) {
                while (isOpModeActive() && Math.abs(getBackLeftPosition()) >= Math.abs(getBackRightPosition())) {
                    setNewPower(-0.2, 0.2);
                    telemetry.addData("Back Left Encoders =" + Math.abs(getBackLeftPosition()));
                    telemetry.addData("Back Right Encoders =" + Math.abs(getBackRightPosition()));
                    telemetry.update();
                }

            } else {
                while (isOpModeActive() && Math.abs(getBackRightPosition()) >= Math.abs(getBackLeftPosition())) {
                    setNewPower(0.2, -0.2);
                    telemetry.addData("Back Left Encoders =" + Math.abs(getBackLeftPosition()));
                    telemetry.addData("Back Right Encoders =" + Math.abs(getBackRightPosition()));
                    telemetry.update();
                }
            }
        }
        setNewPower(0, 0);
        telemetry.addData("Front Left Encoders  =" + getFrontLeftPosition());
        telemetry.addData("Front Right Encoders =" + getFrontRightPosition());
        telemetry.addData("Back Left Encoders  =" + getBackLeftPosition());
        telemetry.addData("Back Right Encoders =" + getBackRightPosition());

    }


    public void gyroStraight( double speed, double distance)
    {
        double currentAngle, error, target_angle=0, p_multiplier=2.5;
        double leftPower=speed, rightPower=speed;
        int loop_samples = 0;
        double leftposition;
        double rightposition;
        double diff;

        telemetry.addData(" ----------- Speed = " + speed);
        telemetry.addData(" ----------- p_multiplier = " + p_multiplier);

        resetAngleToZero();
        resetAllEncoders();
        runtime.reset();
        setNewPower(leftPower,rightPower);

        while ( isOpModeActive() &&
                (    ( Math.abs(getFrontLeftPosition()) < distance )
                  && ( Math.abs(getFrontRightPosition()) < distance )
                )
        )
        {
//            if (runTime.milliseconds() <= 800) {
//                speed = 0.2;
//            }
            loop_samples++;
            currentAngle = getHeading();
            error = (target_angle - currentAngle) / 100;
            error = error * p_multiplier;
            leftPower = range(speed - error);
            rightPower = range(speed + error);


            setNewPower(leftPower, rightPower);
            leftposition= getBackLeftPosition();
            rightposition= getBackRightPosition();
            diff=leftposition-rightposition;

            telemetry.addData( " angle = " + DFpwr.format(currentAngle)
                    + " error = " + DFpwr.format(error)
                    + "   left = " + DFpwr.format(leftPower)
                      + " right = " + DFpwr.format(rightPower)
                      + "    left enc = " +DFenc.format(leftposition)
                    + "    right enc = " +DFenc.format(rightposition)
                    +    " diff enc = " + DFenc.format(diff)
            );

        }  // while

        setNewPower(0,0);
        telemetry.addData("Elapsed Time  =" + runtime.milliseconds());
        if ( speed >= 0) {
            telemetry.addData("FORWARD  Front Left Encoders  =" + getFrontLeftPosition());
            telemetry.addData("FORWARD  Front Right Encoders =" + getFrontRightPosition());
            telemetry.addData("FORWARD  Back Left Encoders  =" + getBackLeftPosition());
            telemetry.addData("FORWARD  Back Right Encoders =" + getBackRightPosition());
        }
        else
        {
            telemetry.addData("BACKWARD  Front Left Encoders =" + getFrontLeftPosition());
            telemetry.addData("BACKWARD  Front Right Encoders =" + getFrontRightPosition());
            telemetry.addData("BACKWARD  Back Left Encoders =" + getBackLeftPosition());
            telemetry.addData("BACKWARD  Back Right Encoders =" + getBackRightPosition());

        }
        telemetry.addData("angle =" + getHeading());
        telemetry.addData("Loop Samples = ", loop_samples);
        telemetry.update();
    }

    public void update() {


        fl = range((turn_y + turn_x - turn_z) * Drive_Power);
        fr = range((turn_y - turn_x + turn_z) * Drive_Power);
        bl = range((turn_y - turn_x - turn_z) * Drive_Power);
        br = range((turn_y + turn_x + turn_z) * Drive_Power);
//        telemetry.addData("fl", fl);
//        telemetry.addData("fr", fr);
//        telemetry.addData("bl", bl);
//        telemetry.addData("br", br);
//        telemetry.addData("Front Left", drive.getFrontLeftPosition());
//        telemetry.addData("Front Right", drive.getFrontRightPosition());
//        telemetry.addData("Back Left", drive.getBackLeftPosition());
//        telemetry.addData("Back Right", drive.getBackRightPosition());
        if ((aButton_pressed == 1) || (bButton_pressed == 1) || (xButton_pressed == 1) || (yButton_pressed == 1)) {
            //If any buttons are pressed then do nothing here
            //Because we will be running the motors in updateCheckMotors
        } else {
            drive.setPower(fl, fr, bl, br);
        }
    }

    public void pressed_A_Down() {
        aButton_pressed = 1;
    }

    public void pressed_A_Up() {
        aButton_pressed = 0;

    }

    public void pressed_B_Down() {
        bButton_pressed = 1;
    }

    public void pressed_B_Up() {
        bButton_pressed = 0;

    }

    public void pressed_X_Down() {
        xButton_pressed = 1;
    }

    public void pressed_X_Up() {
        xButton_pressed = 0;

    }

    public void pressed_Y_Down() {
        yButton_pressed = 1;
    }

    public void pressed_Y_Up() {
        yButton_pressed = 0;

    }

    public void updateCheckMotors() {
//        telemetry.addData("A up 1", aButton_pressed);
        if ((fl != 0) || (fr != 0) || (bl != 0) || (br != 0)) {
            //If user is pushing any joysticks don't run updateCheckMotor
        } else {
            drive.setPower(Button_Power * aButton_pressed, Button_Power * bButton_pressed,
                    Button_Power * xButton_pressed, Button_Power * yButton_pressed);
        }

    }

    public void setPower(double fl, double fr, double bl, double br) {
        drive.setPower(range(fl), range(fr), range(bl), range(br));
    }

    public void stop() {
        drive.stop();
        try {
            myOutWriter.close();
            fOut.close();
        } catch (IOException e) {
            Log.e("Exception", "File close failed " + e.toString());
        }
    }

    private double scaleInput(double dVal) {
        //return range(pow(dVal, 3));

        double[] scaleArray = {0.0, 0.0, 0.01, 0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.08,
                0.12, 0.16, 0.24, 0.30, 0.38, 0.50, 0.70, 0.85, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 18.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 18) {
            index = 18;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;

    }

    private double scaleTurnInput(double dVal) {

        //return range(pow(dVal, 3));

        double[] scaleArray = {0.0, 0.0, 0.01, 0.01, 0.02, 0.02, 0.03, 0.03, 0.04, 0.08,
                0.12, 0.16, 0.24, 0.30, 0.38, 0.38, 0.50, 0.7, 0.90};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 18.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 18) {
            index = 18;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;

    }


    private double range(double val) {
        if (val < -1) val = -1;
        if (val > 1) val = 1;
        return val;
    }

    public double getHeading() {
        return drive.getHeading();
    }

    public void resetAngleToZero() {
        drive.resetAngleToZero();
    }

}
