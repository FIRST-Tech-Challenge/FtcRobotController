package org.firstinspires.ftc.teamcode.Reno.poc;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.time.LocalTime;

public class ConceptTurnPidController extends ConceptMotorController{
    private double Kp = 0.002;                     // factor for "proportional" control
    private double Ki = 0.0001;                     // factor for "integral" control
    private double Kd = 0.0002;                     // factor for "derivative" control

    private double error = 0.0;

    private double feedBackValue = 0.0;

    private double goal = 0.0;

    private double proportional = 0.0;
    private double integral = 0.0;
    private double derivative = 0.0;

    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private ElapsedTime timer = new ElapsedTime();


    public ConceptTurnPidController(double goal)
    {
        this.goal = goal;
    }
    public ConceptTurnPidController(double targetANgle, double Kp, double Ki, double Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.goal = targetANgle;
    }


    public void setFeedbackValue(double value)
    {
        this.feedBackValue = value;
        this.error = this.goal - this.feedBackValue;
    }

    public double getProportional()
    {
        return this.proportional;
    }

    public double getIntegral()
    {
        return this.integral;
    }

    public double getDerivative()
    {
        return this.derivative;
    }

    public void setGoal(double goal)
    {
        this.goal = goal;
        this.error = this.goal - this.feedBackValue;
    }
    public double getError()
    {
        return this.error;
    }
    public void reset()
    {
        this.Kp = 0.0;
        this.Ki = 0.0;
        this.Kd = 0.0;
        this.integral = 0.0;
        this.proportional = 0.0;
        this.derivative = 0.0;
        this.goal = 0.0;
        this.lastError = 0;
        this.lastSlope = 0;
        this.lastTime = 0;

    }
    public double getValue(double currentAngle)
    {
        this.error = this.goal - currentAngle;

        //P
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        proportional = Kp * error;

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        integral = accumulatedError * Ki;
        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        derivative = slope * Kd;

        return 0.1 * Math.signum(error) + 0.9 * Math.tanh(proportional + integral - derivative);

    }

    public double getLastSlope() {
        return lastSlope;
    }
}
