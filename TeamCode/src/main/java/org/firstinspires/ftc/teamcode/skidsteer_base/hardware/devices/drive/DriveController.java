package org.firstinspires.ftc.teamcode.skidsteer_base.hardware.devices.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.framework.AbstractOpMode;
import org.firstinspires.ftc.teamcode.framework.SubsystemController;
import org.firstinspires.ftc.teamcode.framework.userHardware.PIDController;
import org.firstinspires.ftc.teamcode.framework.userHardware.outputs.Logger;

import java.text.DecimalFormat;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.delay;
import static org.firstinspires.ftc.teamcode.framework.AbstractOpMode.isOpModeActive;


public class DriveController extends SubsystemController {

    private Drive drive;
    private PIDController drivePID;

    private double position;

    private double turn_y=0, turn_z=0, l=0, r=0, Drive_Power = 1.0;
    private boolean myTestDriveButtonInProgress = false;

    public ElapsedTime runtime;

    DecimalFormat DF;

    Logger logger;

    public DriveController(){
        init();
    }
    
    public void init(){

        runtime = new ElapsedTime();

        opModeSetup();
        
	    DF = new DecimalFormat("#.##");

        //Put general setup here
        drive = new Drive(hwMap);
        drivePID = new PIDController(.01,0,0);

        if(Mode==mode.Auton){
            //Put Auton setup here
            drive.setSlewSpeed(1);

        }
        else if(Mode==mode.Teleop){
            //Put Teleop setup here
            drive.setSlewSpeed(0.1);

        }

        logger = new Logger("DriveControllerLog.txt");
    }

    public void setY(double y){
        turn_y = y;
        turn_y = (float) scaleInput(turn_y);
    }

    public void setZ(double z){
        turn_z = z;
        turn_z = (float) scaleInput(turn_z);
    }

    public void update(){
        //if button test is running don't drive robot
        if(myTestDriveButtonInProgress == true) {
            return;
        }
        l = range((turn_y + turn_z) * Drive_Power);
        r = range((turn_y - turn_z) * Drive_Power);
        telemetry.addData("turn_y",DF.format(turn_y));
        telemetry.addData("turn_z",DF.format(turn_z));
        telemetry.addData("Drive_Power",Drive_Power);

        telemetry.addData("left",DF.format(l));
        telemetry.addData("right",DF.format(r));
        drive.setPower(l, r);
    }

    public void setPower(double l, double r){
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPower(range(l),range(r));
    }

    public void setPosition(int position, double power){
        this.position = position;
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.setPower(range(power),range(power));
        drive.setPosition(position);
    }

    public boolean isModeDone(){
        telemetry.addData("LeftPos",drive.getLeftPosition());
        telemetry.addData("RightPos",drive.getRightPosition());

        return((abs(drive.getLeftPosition()-position)<10)&&(abs(drive.getRightPosition()-position)<10));
    }

    public void driveDistance(double distance){
        setPosition((int)distance*110,1);
        while (!isModeDone());
    }

    public void driveToMyDistance(double distance,double speed) {

        //if the joystick is being used don't run the code below
        if ((l != 0) || (r !=0))
        {
            return;
        }
        myTestDriveButtonInProgress = true;
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //After setting encoder set to 0
        RobotLog.i("ABCD Left Motor Position after reset encoders  = " + drive.getLeftPosition());
        RobotLog.i("ABCD Right Motor Position after reset encoders = " + drive.getRightPosition());

        logger.log("ABCD Left Motor Position after reset encoders  1 = " + drive.getLeftPosition());
        logger.log("ABCD Right Motor Position after reset encoders 1 = " + drive.getRightPosition());

        while (
                (drive.getLeftPosition()  !=0) ||
                        (drive.getRightPosition() !=0)
                )
        {
            // Sometimes, it will get stuck, so try to reset againh
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Left = " , drive.getLeftPosition());
            telemetry.addData("Right = ", drive.getRightPosition());

            // wait till it equals zero
            // Both motors must be 0 to exit out of the while
        }

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance < 0) {
            speed = -1*speed;
        }

        drive.setPower(range(speed),range(speed));
        telemetry.addData("Drive set power", distance);
        telemetry.update();
        while (( abs(drive.getLeftPosition())<abs(distance))&&
                (abs(drive.getRightPosition())<abs(distance)))
        {
            //wait here and do nothing until motor reaches the distance
        }

        drive.setPower(range(0),range(0));

        telemetry.addData("left Position ", drive.getLeftPosition());
        telemetry.addData("Right Position ", drive.getRightPosition());
        telemetry.update();
        RobotLog.i("ABCD Left Motor Position  = " + drive.getLeftPosition());
        RobotLog.i("ABCD Right Motor Position = " + drive.getRightPosition());

        //delay(5000);
        myTestDriveButtonInProgress = false;

    }
    
    public void driveToMyDistanceGyro(double speed, double distance, double target_angle, double p_multiplier) {

            double     finalLeftTarget;
            double     finalRightTarget;
            double  error;
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while ((drive.getLeftPosition()  !=0) || (drive.getRightPosition() !=0))
            {
                // Sometimes, it will get stuck, so try to reset again
                drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Left = " , drive.getLeftPosition());
                telemetry.addData("Right = ", drive.getRightPosition());

                // wait till it equals zero
                // Both motors must be 0 to exit out of the while
            }

            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(distance < 0) {
                speed = speed*-1;
            }

            finalRightTarget = drive.getRightPosition() + distance;
            finalLeftTarget = drive.getLeftPosition() + distance ;
            // starting  telemetry
            telemetry.addData("driveTo "  +  speed  + " distance "  , distance  );
            telemetry.update();
            //delay(1000);
            drive.setPower(range(speed),range(speed));
            double currentAngle;
            double leftPower = 0;
            double rightPower = 0;
            while (    (abs(drive.getRightPosition()) < abs(finalRightTarget))
                    && (abs( drive.getLeftPosition()) < abs(finalLeftTarget ))
                  )
            {
                //If you are turning left, it is a positive angle
                //If you are turning Right, it is a negative angle

                //sample 1   = -20 - 0 = -20
                //sample 2   = 20 - 0 = 20
                currentAngle = getHeading();
                error = (target_angle - currentAngle)/100;
                error = error * p_multiplier;
                //        Sample 1  .5 - (-20) = 20.5        .5 +(-20) = -19.5
                //        Sample 2  .5 - 20 = -19.5          .5 + 20 = 20.5
                drive.setPower( range(speed-error), range(speed+error) );
                leftPower = (speed-error);
                rightPower = (speed+error);
                logger.log("Error = " + error + "  Angle1 = " + currentAngle);
            }
            telemetry.addData("L "  +  drive.getLeftPosition()  + " R "  , drive.getRightPosition()
                    + " L " + leftPower + " R " + rightPower);
            drive.setPower(0,0);


            telemetry.update();
          //  delay(1000);
    }

    public void measureEncoders(int LeftOrRight,double milliSecondsStop ) {

        double LeftValue1,LeftValue2,LeftValue3;
        double RightValue1,RightValue2,RightValue3;

        logger.log("ABCD BEGIN  measureEncoders" );
        //if the joystick is being used don't run the code below
        if ((l != 0) || (r !=0))
        {
            return;
        }
        myTestDriveButtonInProgress = true;
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //After setting encoder set to 0
        RobotLog.i("measureEncoders Left  = " + drive.getLeftPosition());
        RobotLog.i("measureEncoders Right = " + drive.getRightPosition());

        logger.log("Start Left   = " + drive.getLeftPosition());
        logger.log("Start Right  = " + drive.getRightPosition());

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (LeftOrRight == 0 )
        {
            logger.log(" MEASURING LEFT MOTOR" );
        }
        else
        {
            logger.log(" MEASURING RIGHT MOTOR" );
        }

        runtime.reset();
        if (LeftOrRight == 0 ) {
            drive.setPower(range(1), range(0));
        }else
        {
            drive.setPower(range(0), range(1));
        }

        while ( runtime.milliseconds() <= milliSecondsStop )
        {
            // allow motors to run until time is reached
        }

        drive.setPower(range(0), range(0));

        LeftValue1 = drive.getLeftPosition();
        RightValue1 = drive.getRightPosition();

        delay(600);
        LeftValue2 = drive.getLeftPosition();
        RightValue2 = drive.getRightPosition();

        delay(600);
        LeftValue3 = drive.getLeftPosition();
        RightValue3 = drive.getRightPosition();

        logger.log("end left measure  1 = " +  LeftValue1 );
        logger.log("end Right measure 1 = " + RightValue1 );

        logger.log("end left measure  2 = " +  LeftValue2 );
        logger.log("end Right measure 2 = " + RightValue2 );

        logger.log("end left measure  3 = " +  LeftValue3 );
        logger.log("end Right measure 3 = " + RightValue3 );


        myTestDriveButtonInProgress = false;
        logger.log("END measureEncoders \n" );



    }

    public void turnTo(double angle, double speed, int error, int period){
        double power;
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(isOpModeActive()) {
            //error is the percentage deviation.
            while (((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)&&AbstractOpMode.isOpModeActive()) {
                if(angle-getHeading()>180) power = drivePID.output(angle, 180-getHeading());
                else power = drivePID.output(angle, getHeading());
                drive.setPower(-power*speed, power*speed);
                telemetry.addDataDS("--------------------");
                telemetry.addData("Heading: ", getHeading());
                telemetry.addData("Power: ", getHeading());
                telemetry.update();
            }
            runtime.reset();
            while (runtime.milliseconds()<period){
                if((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error)break;
            }
            drive.setPower(0,0);
            if((abs(getHeading() - angle)) > error && (abs(getHeading() + angle)) > error) continue;
            telemetry.addData("Final heading: ", getHeading());
            telemetry.update();
            return;
        }
    }

    public void turnToVer2(double angle, double speed, int error){
        //if the joystick is being used don't run the code below
        if ((l != 0) || (r !=0))
        {
            return;
        }
        myTestDriveButtonInProgress = true;

        double myPID = 1.0;
        double power;
        double initialAngle = getHeading();

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // 1.0 is the tolerance
        while((abs(getHeading()-angle))>2.0) {
            // this will keep declaring power
            //double power = drivePID.output(angle,getHeading());
                                /*0 degrees
                                      |
                        \                           /
                   45 degrees      Front       -45 degrees            */
            //power = drivePID.output(angle,getHeading());
            //calculate how muc power I need to apply
            power = myPID * (getHeading() - angle) / 100;
            if (power > 1.0)
            {
                power = 1.0;
            }
            if (power < -1.0)
            {
                power = -1.0;
            }
            telemetry.addData("power", DF.format(power));
            //getHeading() is current angle
            if(abs(angle-getHeading())*.01>.05) {
                // turn right or left
                // if power = .5, right turn
                // if power = -.5 left turn
                drive.setPower(power,-power);
            }



            telemetry.addData("target angle", angle);
            // Heading is the angle measurement now
            telemetry.addData("Heading",getHeading());
            telemetry.update();
        }// end of while  loop

        telemetry.addData("Finish Heading",getHeading());
        telemetry.update();
        drive.setPower(0,0);
        myTestDriveButtonInProgress = false;
    }

    public double getHeading(){
        return drive.getHeading();
    }

    public void resetAngleToZero() {
        drive.resetAngleToZero();
    }

    public void stop(){
        drive.stop();

        logger.stop();
    }

    private double scaleInput(double val) {
        return range(pow(val, 3));
    }

    private double range(double val){
        if(val<-1) val=-1;
        if(val>1) val=1;
        return val;
    }
}
