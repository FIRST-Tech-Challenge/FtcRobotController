package org.firstinspires.ftc.team6220_2020.ResourceClasses;

/**
 Proportional-Integral-Derivative Filter
 Calculates an input difference, it's first integral, and first derivative.
 Generally used to produce efficient, non-oscillating motion in a one dimensional system.

 INPUTS:
 -(once) PID coefficients (tuning values)
 -Difference between the target value and current value. (e.g a target angle vs encoder position)

 OUTPUTS:
 -First derivative control. (e.g. a motor power)
 */

public class PIDFilter implements Filter
{
    // Proportional coefficient
    private double P;
    // Integral coefficient
    private double I;
    // Derivative coefficient
    private double D;

    // Construct filter with the coefficients
    public PIDFilter(double P, double I, double D)
    {
        this.P = P;
        this.I = I;
        this.D = D;
    }


    public double[] values = new double[2];
    public double sum = 0;
    public double dV  = 0;


    // Update with new value
    public void roll(double newValue)
    {
        // Update calculated values
        sum += values[0];
        dV = values[0] - values[1];

        // Introduce new value
        values[1] = values[0];
        values[0] = newValue;
    }


    public double getFilteredValue()
    {
        return (P * values[0] ) + ( I * sum) + ( D * dV );
    }
}
