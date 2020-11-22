package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    private double distanceFromGoal;
    private double lastDistanceFromGoal;
    private double integral;
    private double kP;
    private double kI;
    private double kD;

    private double pComponent;
    private double iComponent;
    private double dComponent;

    private ElapsedTime e = new ElapsedTime();
    private double elapsedTime;

    /**
     * @param error the error of the controller
     * @param p     The proportional gain.
     * @param i     The integral gain.
     * @param d     The derivative gain.
     */
    public PID(double error, double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        distanceFromGoal = error;
        e.startTime();
    }

    /**
     * Updates the PID controller, this should be called every before the controller is used
     *
     * @param error the error of the controller
     */
    public void update(double error) {
        setDistanceFromGoal(error);
        elapsedTime = e.time();
        integral += distanceFromGoal * elapsedTime;
        pComponent = p();
        iComponent = i();
        dComponent = d();
        e.reset();
        lastDistanceFromGoal = distanceFromGoal;
    }

    //sets the value of the distance from goal for the controller
    private void setDistanceFromGoal(double d) {
        distanceFromGoal = d;
    }

    //calculates the proportional component
    public double p() {
        return distanceFromGoal * kP;
    }

    //calculates the integral component
    public double i() {
        return integral * kI;
    }

    //calculates the derivative component
    public double d() {
        return (distanceFromGoal - lastDistanceFromGoal) / elapsedTime * kD;
    }

    /**
     * @return Returns the output of the PID controller
     */
    public double getPID() {
        return pComponent + iComponent + dComponent;
    }

    public void resetPid() {
        lastDistanceFromGoal = 0;
        integral = 0;
    }

}
