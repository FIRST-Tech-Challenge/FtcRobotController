package org.firstinspires.ftc.team417_2020.Resources;

/*
    Finite Impulse Response Filter
    Calculates a weighted average based on the age of input values.
    Used to prevent or induce certain output frequencies

    INPUTS:
      -Control value (e.g. joystick value)

    OUTPUTS:
      -Filtered value (e.g. a smoother, non-step function)
*/

public class FIRFilter implements Filter
{
    public double[] values;
    private double[] weights;

    public FIRFilter(double[] w)
    {
        weights = w;
    }

    // Initialize with weight as a polynomial function of age
    public FIRFilter(Polynomial func, int depth)
    {
        weights = SequenceUtilities.arrayFromPolynomial(0,depth,func);
    }

    // Set up with new value set
    public void roll(double newValue)
    {
        SequenceUtilities.roll(values,newValue);
    }

    public double getFilteredValue()
    {
        return SequenceUtilities.weightedAverage(values, weights);
    }

    public static double[] createTriangularFilter(int length) {
        double[] filterOut = new double[length];
        for (int i = 0; i < length; i++) {
            filterOut[i] = length - i;
        }
        return filterOut;
    }
}
