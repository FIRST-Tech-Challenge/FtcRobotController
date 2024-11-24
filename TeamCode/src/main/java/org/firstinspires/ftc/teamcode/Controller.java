package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;
import static org.firstinspires.ftc.teamcode.Utilize.SigNum;

public class Controller {
    public double Kp, Ki, Kd, Kf, baseSpeed, Setpoint, Error, LastError, ErrorTolerance;
    public double Integral, Derivative, Dt, LastTime, BaseSpeed;

    public Controller(double Kp, double Ki, double Kd, double Kf, double baseSpeed, double errorTolerance) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.baseSpeed = Range.clip(Math.abs(baseSpeed), 0, 0.5);
        Setpoint = Dt = Error = Integral = Derivative = LastTime = LastError = 0;
        this.ErrorTolerance = Math.abs(errorTolerance);
    }

    public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public double Calculate(double setpoint, double current) {
        Setpoint = setpoint;
        return Calculate(setpoint - current);
    }

    public double Calculate(double error) {
        double CurrentTime = System.nanoTime() * 1E-9;
        if (LastTime == 0) LastTime = CurrentTime;
        this.Dt         = CurrentTime - LastTime;
        this.LastTime    = CurrentTime;

        this.Error       = error;  // Error = Setpoint - Current
        boolean Is_Error_In_Tolerance = atSetpoint();
        if (Is_Error_In_Tolerance) {
            this.Integral = 0;
            return 0;
        }
        this.Integral    = Integral + (Error * Dt);
        this.Integral    = Range.clip(Integral, -1, 1);
        this.Derivative  = Math.abs(Dt) > 1E-6 ? (Error - LastError) / Dt : 0;
        this.LastError   = Error;
        this.BaseSpeed   = baseSpeed * SigNum(error);
        return (this.Error * this.Kp) + (this.Integral * this.Ki) + (this.Derivative * this.Kd) + (this.Setpoint * this.Kf);
    }

    public boolean atSetpoint() { return Math.abs(this.Error) < this.ErrorTolerance; }
}
