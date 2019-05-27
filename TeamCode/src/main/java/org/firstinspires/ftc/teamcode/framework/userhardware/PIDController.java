package org.firstinspires.ftc.teamcode.framework.userhardware;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.framework.userhardware.DoubleTelemetry.LogMode.INFO;

public class PIDController {
    private double p, i, d, iVal, lastError = 0, ilimit = 1, minimumOutput = 0;
    private boolean logging = false;
    private DecimalFormat DF;

    public PIDController() {
        this(1, 1, 1);
        DF = new DecimalFormat("#.##");
    }

    public PIDController(double P, double I, double D) {
        this(P, I, D, 1);
        DF = new DecimalFormat("#.##");
        logControl(INFO, " P = " + P + " I = " + I + " D = " + D);

    }

    public PIDController(double P, double I, double D, double Ilimit) {
        p = P;
        i = I;
        d = D;
        ilimit = Ilimit;
        DF = new DecimalFormat("#.##");
        logControl(INFO, " P = " + P + " I = " + I + " D = " + D);

    }

    public PIDController(double P, double I, double D, double Ilimit, double minOutput) {
        p = P;
        i = I;
        d = D;
        ilimit = Ilimit;
        minimumOutput = Math.abs(minOutput);
        DF = new DecimalFormat("#.##");
        logControl(INFO, " P = " + P + " I = " + I + " D = " + D);
    }

    public void setMinimumOutput(double minOutput) {
        minimumOutput = Math.abs(minOutput);
    }

    public void setLogging(boolean logging) {
        this.logging = logging;
    }

    public double getError() {
        return lastError;
    }

    public void logControl(DoubleTelemetry.LogMode mode, String line) {
        //AbstractOpMode.getTelemetry().addData(mode, line);
    }

    public double output(double target, double current) {
        double pvar;
        double ivar;
        double dvar;
        double error = target - current, out;
        pvar = PTerm(error);
        ivar = ITerm(error);
        dvar = DTerm(error);
        out = pvar + ivar + dvar;
        logControl(INFO,"Cur Err = " + DF.format(error) +" Last Error = " + DF.format(lastError)
                + " pvar = " + DF.format(pvar) + " ivar = " + DF.format(ivar) + " dvar = " + DF.format(dvar));
        lastError = error;
        if (out > 0 && out < minimumOutput) out = minimumOutput;
        if (out < 0 && out > -minimumOutput) out = -minimumOutput;
        return out;
    }

    private double PTerm(double error) {
        if (logging)
            AbstractOpMode.getTelemetry().addData(INFO, "P", error * (p / 1000));
        return error * (p / 1000);
    }

    private double ITerm(double error) {
        iVal = (iVal + error) * (i / 1000);
        if (iVal < -ilimit) iVal = -ilimit;
        if (iVal > ilimit) iVal = ilimit;
        if (logging) AbstractOpMode.getTelemetry().addData(INFO, "I", iVal);
        return iVal;
    }

    private double DTerm(double error) {
        if (logging)
            AbstractOpMode.getTelemetry().addData(INFO, "D", (error - lastError) * (d / 1000));
        return (error - lastError) * (d / 1000);
    }

    public void reset() {
        lastError = 0;
        iVal = 0;
    }
}