package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.ArrayList;
import java.util.List;

/**
 * Collection of utility methods for feedforward constan "tuning."
 */
public class TuningUtil {

    /**
     * Results from a ramp test.
     * @see TuningUtil#fitRampData(List, List, List, boolean)
     */
    public static class RampFFResult {
        public final double kV, kStatic, rSquared;

        public RampFFResult(double kV, double kStatic, double rSquared) {
            this.kV = kV;
            this.kStatic = kStatic;
            this.rSquared = rSquared;
        }
    }

    /**
     * Results from a constant power test.
     * @see TuningUtil#fitConstantPowerData(List, List, double, double, double) 
     */
    public static class AccelFFResult {
        public final double kA, rSquared;

        public AccelFFResult(double kA, double rSquared) {
            this.kA = kA;
            this.rSquared = rSquared;
        }
    }

    private TuningUtil() {

    }

    /**
     * Numerically compute dy/dx from the given x and y values. The returned list is padded to match
     * the length of the original sequences.
     * @param x x-values
     * @param y y-values
     * @return derivative values
     */
    public static List<Double> numericalDerivative(List<Double> x, List<Double> y) {
        List<Double> deriv = new ArrayList<>();
        for (int i = 2; i < x.size(); i++) {
            deriv.add((y.get(i) - y.get(i-2)) / (x.get(i) - x.get(i-2)));
        }
        deriv.add(0, deriv.get(0));
        deriv.add(deriv.get(deriv.size() - 1));
        return deriv;
    }

    /**
     * Fit data from a "ramp" test where the power is linearly increased (minimal acceleration).
     * This data can be used to compute the feedforward velocity and static constants.
     * @param timeSamples time samples in seconds
     * @param positionSamples motor positions in real distance units (not encoder ticks)
     * @param powerSamples power samples in the range [0.0, 1.0]
     * @param fitKStatic true if kStatic should be fit (note: this affects the kV computation)
     * @return kV and kStatic along with R^2 from the underlying regression
     */
    public static RampFFResult fitRampData(List<Double> timeSamples, List<Double> positionSamples,
                                           List<Double> powerSamples, boolean fitKStatic) {
        List<Double> velocitySamples = numericalDerivative(timeSamples, positionSamples);

        SimpleRegression rampReg = new SimpleRegression(fitKStatic);
        for (int i = 0; i < velocitySamples.size(); i++) {
            rampReg.addData(velocitySamples.get(i), powerSamples.get(i));
        }
        return new RampFFResult(rampReg.getSlope(), rampReg.getIntercept(), rampReg.getRSquare());
    }

    /**
     * Fit data from a constant power test where there is generally more substantial acceleration.
     * This data can be used to determine the feedforward acceleration constant given values for the
     * feedforward velocity and static constants.
     * @param timeSamples time samples in seconds
     * @param positionSamples motor positions in real distance units (not encoder ticks)
     * @param power power value in the range [0.0, 1.0]
     * @param kV feedforward velocity constant
     * @param kStatic feedforward static constant
     * @return kA and R^2 from the underlying regression
     */
    public static AccelFFResult fitConstantPowerData(List<Double> timeSamples, List<Double> positionSamples,
                                                     double power, double kV, double kStatic) {
        List<Double> velocitySamples = numericalDerivative(timeSamples, positionSamples);
        List<Double> accelerationSamples = numericalDerivative(timeSamples, velocitySamples);

        SimpleRegression constPowerReg = new SimpleRegression(false);
        for (int i = 0; i < accelerationSamples.size(); i++) {
            double velocityPower = kV * velocitySamples.get(i);
            if (Math.abs(velocityPower) > 1e-2) {
                velocityPower += Math.signum(velocityPower) * kStatic;
            } else {
                velocityPower = 0;
            }
            double accelerationPower = power - velocityPower;
            constPowerReg.addData(accelerationSamples.get(i), accelerationPower);
        }
        return new AccelFFResult(Math.signum(power) * constPowerReg.getSlope(), constPowerReg.getRSquare());
    }

}
