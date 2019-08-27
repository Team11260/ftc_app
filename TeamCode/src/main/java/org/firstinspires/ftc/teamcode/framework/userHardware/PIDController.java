package org.firstinspires.ftc.teamcode.framework.userhardware;

public class PIDController {
    private double p,i,d,iVal,lastError=0;

    public PIDController(){
        p = 1;
        i = 1;
        d = 1;
    }

    public PIDController(double P, double I, double D){
        p = P;
        i = I;
        d = D;
    }

    public double output(double target, double current){
       double error=target-current, out;
        out = PTerm(error)+ITerm(error)+DTerm(error);
        lastError = error;
        return out;
    }

    private double PTerm(double error){
        return error*p;
    }

    private double ITerm(double error){
        return iVal=(iVal+error)/i;
    }

    private double DTerm(double error){
        return (error-lastError)*d;
    }
}