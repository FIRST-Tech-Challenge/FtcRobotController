package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import android.util.Range;

import static org.firstinspires.ftc.teamcode.util.Conversions.between;

/**
 * Created on 6/28/2015.
 */
//----------------------------------------------------------------------------
// Copyright (c) FIRST 2008. All Rights Reserved.
// Open Source Software - may be modified and shared by FRC teams. The code
// must be accompanied by the FIRST BSD license file in the root directory of
// the project.
//
// File: FL_PIDController.java
//
// Description: This PIDCOntroller class is a port from the FIRST PID Controller
//              task. This port removes the additional thread overhead but it
//              must run within a periodic task (not a continous task) to work
//              correctly.
//
// Lead: Mark
//              This version was published by FRC Team 443 here:
//              https://code.google.com/p/frcteam443/source/browse/trunk/2010_Post_Season/Geisebot/src/freelancelibj/PIDController.java?r=17
//
// Iron Reign has customized this for Android and added time sampling so that these
// calculations are not reliant on being called upon a completely regular period
// This also isolates the PID constants from the rate at which the calculations are made
// so we are free to vary the amount of sleep in the calling loop without requiring a change in the
// constants -- as long as the period is small enough to meet the requirements of the physical system under control
// The PID constants should now be scaled to the real-time dynamics of the physical system and not
// so much to the frequency of the calculations
// a pretty definitive look at timing on android suggests using System.nanoTime:
// http://gamasutra.com/view/feature/171774/getting_high_precision_timing_on_.php?print=1
// Iron Reign has also added integral windup prevention by resetting accumulated error on zero crossings and when current error is larger than the integral cut-in
// An example of integral cut-in might be +-5 degrees (depending on physical sysatem) when steering based on an IMU
// A summary of windup issues can be found at https://en.wikipedia.org/wiki/Integral_windup
// ----------------------------------------------------------------------------


public class PIDController {

    private double m_P;                 // factor for "proportional" control
    private double m_I;                 // factor for "integral" control
    private double m_D;                 // factor for "derivative" control
    private double m_input;             // sensor input for pid controller
    private double m_maximumOutput = 1.0;       // |maximum output|
    private double m_minimumOutput = -1.0;      // |minimum output|
    private double m_maximumInput = 0.0;                // maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;                // minimum input - limit setpoint to this
    private boolean m_continuous = false;       // do the endpoints wrap around? eg. Absolute encoder or imu
    private boolean m_enabled = false;                  //is the pid controller enabled
    private double m_integralCutIn = 0.0; //this is the value for instantaneos error at which integral
    private boolean m_integralCrossingReset = true; //when enabled, zero total error when error crosses zero
    private double m_prevError = 0.0;   // the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0; //the sum of the errors for use in the integral calc
    private double m_deltaError = 0.0; //the latest change in error
    private double m_tolerance = 0.05;  //the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;
    private long m_prevTime; //time of previous calculate() in nanoseconds from the current epoch
    private double m_deltaTime; // time between calls to calculate() in fractional seconds
    private long m_currentTime;
    private double pwrP = 0.0;
    private double pwrI = 0.0;
    private double pwrD = 0.0;


    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {

        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
        m_prevTime=System.nanoTime();
    }


    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate() {

        // If enabled then proceed into controller calculations
        if (m_enabled) {

            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // !!!!DEBUG!!!
            //System.out.println(m_setpoint);
            //Log.d("PID", String.valueOf(m_setpoint));

            // If continuous is set to true allow wrap around
            if (m_continuous) {
                if (Math.abs(m_error) >
                        (m_maximumInput - m_minimumInput) / 2) {
                    if (m_error > 0) {
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    } else {
                        m_error = m_error +
                                m_maximumInput - m_minimumInput;
                    }
                }
            }


            //time since last iteration
            m_currentTime = System.nanoTime();
            m_deltaTime=(m_currentTime-m_prevTime)/1E9;
            m_prevTime=m_currentTime;

            if(m_deltaTime > .15)
            {
                Log.e("", "Laggy Loop! " + m_deltaTime  + "  sec");
                //m_deltaTime = 0;
            }

            //integrate with windup prevention
            //reset total error if we are outside the integral control regime or if current error crosses zero
//            if (!(Math.abs(m_integralCutIn) < .00000000001)) { //this is just a check if integralCutIn is zero - zero actually means we ignore cut-in and allow total error to accumulate regardless of current error magnitude
                if (Math.abs(m_integralCutIn) < Math.abs(m_error)){ //we are outside of intended integral regime - reset accumulated error
                    m_totalError = 0.0;
            }

            if(between(m_I * m_totalError, m_minimumOutput, m_maximumOutput)){ //extra sanity check - only accumulate error if the integral output on it's own is inside the output range
                //integral calculation factored for time
                m_totalError += (m_error * m_deltaTime);
            }

            //also reset accumulated error on zero crossing
            if (m_integralCrossingReset) {
                if (m_error * m_prevError <= 0.0) //current error and previous error have different signs when crossing zero regardless of direction
                    m_totalError = 0;
            }

            //derivative calculation factored for time
            m_deltaError = (m_error - m_prevError) * m_deltaTime;

            // Perform the primary PID calculation

            pwrP = m_P * m_error;
            pwrI = m_I * m_totalError;
            pwrD = m_D * m_deltaError;

            m_result = pwrP + pwrI + pwrD;

            // Set the current error to the previous error for the next cycle
            m_prevError = m_error;

            // Make sure the final result is within bounds
            if (m_result > m_maximumOutput) {
                m_result = m_maximumOutput;
            } else if (m_result < m_minimumOutput) {
                m_result = m_minimumOutput;
            }
        }
    }

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d) {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public synchronized double getP() {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public synchronized double getI() {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public synchronized double getD() {
        return m_D;
    }


    /**
     * Return the current PID result
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID() {
        calculate();
        return m_result;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather than using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous) {
        m_continuous = continuous;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather than using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    public void setContinuous() {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input
     * @param maximumInput the maximum value expected from the output
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    /**
     * Sets the regime around zero error in which errors will be integrated
     * This assumes that we want to prevent integral wind-up while the proportional constant can do the correction
     * But when the error is small, the proportional response is weak and the integral should come into play
     * if zero, then the error will always be integrated regardless of current error size - almost guaranteed to lead to wind-up
     * @param cutIn
     */
    public void setIntegralCutIn(double cutIn){
        m_integralCutIn = cutIn;
    }

    /**
     * By default integral zero-crossing windup is enabled - this will help prevent oscillations around the setpoint
     * This is really here only so it can be turned of for special cases
     * @param reset
     */
    public void enableIntegralZeroCrossingReset(boolean reset){
        m_integralCrossingReset = reset;
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint) {
        if (m_maximumInput > m_minimumInput) {
            if (setpoint > m_maximumInput) {
                m_setpoint = m_maximumInput;
            } else if (setpoint < m_minimumInput) {
                m_setpoint = m_minimumInput;
            } else {
                m_setpoint = setpoint;
            }
        } else {
            m_setpoint = setpoint;
        }
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError() {
        return m_error;
    }

    public synchronized double getTotalError() {
        return m_totalError;
    }

    public synchronized void setTotalError( double totalError ) { m_totalError = totalError; }

    public synchronized double getDeltaError() {
        return m_deltaError;
    }

    public synchronized double getDeltaTime() {
        return m_deltaTime;
    }

    public synchronized double getPwrP () {
        return pwrP;
    }

    public synchronized double getPwrI () {
        return pwrI;
    }

    public synchronized double getPwrD () {
        return pwrD;
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */
    public void setTolerance(double percent) {
        m_tolerance = percent;
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This asssumes that the maximum and minimum input
     * were set using setInput.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget() {
        return (Math.abs(m_error) < m_tolerance / 100 *
                (m_maximumInput - m_minimumInput));
    }

    /**
     * Begin running the PIDController
     */
    public void enable() {

        if (!m_enabled) m_prevTime=System.nanoTime(); //if it's been false for a while, the previous time is likely very stale
        m_enabled = true;
    }

    /**
     * Stop running the PIDController, this sets the output to zero before stopping.
     */
    public void disable() {

        m_enabled = false;
    }

    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public void reset() {
        disable();
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
        m_prevTime=System.nanoTime();
    }

    public void setInput(double input){
        m_input = input;
    }

}