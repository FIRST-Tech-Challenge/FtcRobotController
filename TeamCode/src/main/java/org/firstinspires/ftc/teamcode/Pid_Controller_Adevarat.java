package org.firstinspires.ftc.teamcode;

// PID controller courtesy of Peter Tischler, with modifications.
public class Pid_Controller_Adevarat
{
    private double m_P;                     // factor for "proportional" control
    private double m_I;                     // factor for "integral" control
    private double m_D;                     // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;	// |maximum output|
    private double m_minimumOutput = -1.0;	// |minimum output|
    private double m_maximumInput = 0.0;	// maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;	// minimum input - limit setpoint to this
    private boolean m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;      // is the pid controller enabled
    private double m_prevError = 0.0;
    private double m_prevPrevError= 0.0;// the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_tolerance = 0.01;      // the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;


    public double getDError(){ return m_prevPrevError - m_error; }
    public double getISum(){
        return m_totalError;
    }
    public Pid_Controller_Adevarat(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate()
    {
        int sign = 1;
        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            m_error = m_setpoint - m_input;
            // Calculate the error signal

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    else
                        m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput) && (Math.abs(m_error) > Math.abs(m_tolerance)))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevPrevError = m_prevError;
            m_prevError = m_error;

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
        }
    }

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d)
    {
        if(i != m_I){
            m_totalError = 0;
        }
        m_P = p;
        m_I = i;

//        //normalising the total error
//        if(m_totalError * m_I < m_minimumOutput && m_I != 0.0){
//            m_totalError = m_minimumOutput / m_I;
//        }
//        else if(m_totalError * m_I > m_maximumOutput && m_I != 0.0){
//            m_totalError = m_maximumOutput / m_I;
//        }
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP()
    {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI()
    {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD()
    {
        return m_D;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID()
    {
        calculate();
        return m_result;
    }

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID(double input)
    {
        setInput(input);
        return performPID();
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous)
    {
        m_continuous = continuous;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    public void setContinuous()
    {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    public void setInputRange(double minimumInput, double maximumInput)
    {
        m_minimumInput = Math.abs(minimumInput);
        m_maximumInput = Math.abs(maximumInput);
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = Math.abs(minimumOutput);
        m_maximumOutput = Math.abs(maximumOutput);
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public void setSetpoint(double setpoint)
    {
        int     sign = 1;

        if (m_maximumInput > m_minimumInput)
        {
            if (setpoint < 0) sign = -1;

            if (Math.abs(setpoint) > m_maximumInput)
                m_setpoint = m_maximumInput * sign;
            else if (Math.abs(setpoint) < m_minimumInput)
                m_setpoint = m_minimumInput * sign;
            else
                m_setpoint = setpoint;
        }
        else
            m_setpoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public double getSetpoint()
    {
        return m_setpoint;
    }

    /**
     * Retruns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError()
    {
        return m_error;
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */
    public void setTolerance(double percent)
    {
        m_tolerance = percent;
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget()
    {
        m_error = m_setpoint - m_input;
        return (Math.abs(m_error) < Math.abs(m_tolerance));
    }

    /**
     * Begin running the PIDController
     */
    public void enable()
    {
        m_enabled = true;
    }

    /**
     * Stop running the PIDController.
     */
    public void disable()
    {
        m_enabled = false;
    }

    public boolean enabled() {return m_enabled;}

    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public void reset()
    {
        disable();
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
    }

    /**
     * Set the input value to be used by the next call to performPID().
     * @param input Input value to the PID calculation.
     */
    public void setInput(double input)
    {
        int sign = 1;

        if (m_maximumInput > m_minimumInput)
        {
            if (input < 0) sign = -1;

            if (Math.abs(input) > m_maximumInput)
                m_input = m_maximumInput * sign;
            else if (Math.abs(input) < m_minimumInput)
                m_input = m_minimumInput * sign;
            else
                m_input = input;
        }
        else
            m_input = input;
    }
}