package com.SCHSRobotics.HAL9001.util.math.geometry;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.io.Serializable;

/**
 * A class representing a complex number of the form a+bi
 */
public class ComplexNumber implements Serializable {
    //A vector containing the real and imaginary components of the complex number.
    private final Vector2D vector;

    /**
     * Creates a complex number from a real and imaginary part.
     *
     * @param a The real part of the complex number.
     * @param b The imaginary part of the complex number.
     */
    public ComplexNumber(double a, double b) {
        vector = new Vector2D(a, b);
    }

    /**
     * Creates a complex number from a vector containing a real and imaginary part.
     *
     * @param vector A vector with the real part of the number as the x component and the imaginary part as the y component.
     */
    public ComplexNumber(Vector2D vector) {
        this.vector = vector;
    }

    /**
     * Makes a copy of the complex number.
     *
     * @return A copy of the complex number.
     */
    public ComplexNumber copy() {
        return new ComplexNumber(vector.copy());
    }

    /**
     * Rounds the complex number to the given number of decimal places.
     *
     * @param places The number of decimal places to round to.
     * @return A copy of this complex number with real and imaginary parts rounded to the specified number of decimal places.
     */
    public ComplexNumber round(int places) {
        return new ComplexNumber(HALMathUtil.round(Re(), places), HALMathUtil.round(Im(), places));
    }

    /**
     * Returns whether this complex number is 0.
     *
     * @return Whether this complex number is 0.
     */
    public boolean isZero() {
        return vector.isZeroVector();
    }

    /**
     * Returns whether this complex number is real.
     *
     * @return Whether this complex number is real.
     */
    public boolean isReal() {
        return Im() == 0;
    }

    /**
     * Returns whether this complex number is imaginary.
     *
     * @return Whether this complex number is imaginary.
     */
    public boolean isImaginary() {
        return Re() == 0;
    }

    /**
     * Calculates the absolute value of this complex number.
     *
     * @return The absolute value of this complex number.
     */
    public double abs() {
        return vector.magnitude();
    }

    /**
     * Converts this complex number to a real number, if possible.
     *
     * @throws ArithmeticException Throws this exception if the complex number is not real.
     * @return The real number version of this complex number.
     */
    public double toReal() {
        ExceptionChecker.assertTrue(isReal(), new ArithmeticException("Cannot convert to real number. This number is not real."));
        return Re();
    }

    /**
     * Gets the real part of this complex number.
     *
     * @return The real part of this complex number.
     */
    public double Re() {
        return vector.getX();
    }

    /**
     * The imaginary part of this complex number.
     *
     * @return The imaginary part of this complex number.
     */
    public double Im() {
        return vector.getY();
    }

    /**
     * Sets the real part of this complex number.
     *
     * @param re The new real part of this complex number.
     */
    public void setRe(double re) {
        vector.setX(re);
    }

    /**
     * Sets the imaginary part of this complex number.
     *
     * @param im The new imaginary part of this complex number.
     */
    public void setIm(double im) {
        vector.setY(im);
    }

    /**
     * Gets the r value of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @return The r value of the complex number in polar (rcis(theta)) form.
     */
    public double getR() {
        return vector.magnitude();
    }

    /**
     * Gets the r^2 value of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @return The r^2 value of the complex number in polar (rcis(theta)) form.
     */
    public double getRSquared() {
        return vector.magnitudeSquared();
    }

    /**
     * Gets the theta value in radians of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @return The theta value (in radians) of the complex number in polar (rcis(theta)) form.
     */
    public double getTheta() {
        return vector.getTheta();
    }

    /**
     * Gets the theta value of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @param angleUnit The units that theta will be returned in.
     * @return The theta value of the complex number in polar (rcis(theta)) form.
     */
    public double getTheta(HALAngleUnit angleUnit) {
        return vector.getTheta(angleUnit);
    }

    /**
     * Sets the r value of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @param r The new r value for the current complex number in polar form.
     */
    public void setR(double r) {
        vector.scaleTo(r);
    }

    /**
     * Sets the theta value in radians of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @param thetaRadians The new theta value in radians for the current complex number in polar form.
     */
    public void setTheta(double thetaRadians) {
        vector.setTheta(thetaRadians);
    }

    /**
     * Sets the theta value of the current complex number when it is converted to rcis(theta) form.
     * Ironic because the person who wrote this function is not cis.
     *
     * @param theta The new theta value for the current complex number in polar form.
     * @param angleUnit The units for the new theta value.
     */
    public void setTheta(double theta, HALAngleUnit angleUnit) {
        vector.setTheta(theta, angleUnit);
    }

    /**
     * Gets the complex conjugate of the current complex number.
     *
     * @return The complex conjugate of the current complex number.
     */
    public ComplexNumber conjugate() {
        return new ComplexNumber(Re(), -Im());
    }

    /**
     * Converts the complex number to a 2d vector with the real part as the x component
     * and the imaginary part as the y component.
     *
     * @return A 2d vector with the real part as the x component and the imaginary part as the y component.
     */
    public Vector2D toVector() {
        return vector;
    }

    /**
     * Adds two complex numbers.
     *
     * @param c The complex number being added to the current complex number.
     * @return A new complex number equal to the sum of this complex number and the given complex number.
     */
    public ComplexNumber add(@NotNull ComplexNumber c) {
        return new ComplexNumber(vector.add(c.vector));
    }

    /**
     * Adds a complex number and real number.
     *
     * @param real A real number being added to this complex number.
     * @return A new complex number equal to the sum of this complex number and the given real number.
     */
    public ComplexNumber add(double real) {
        return add(fromReal(real));
    }

    /**
     * Subtracts two complex numbers.
     *
     * @param c The complex number being subtracted from the current complex number.
     * @return A new complex number equal to this complex number minus the given complex number.
     */
    public ComplexNumber subtract(@NotNull ComplexNumber c) {
        return new ComplexNumber(vector.subtract(c.vector));
    }

    /**
     * Subtracts a complex number and a real number.
     *
     * @param real A real number being subtracted from this complex number.
     * @return A new complex number equal to this complex number minus the given complex number.
     */
    public ComplexNumber subtract(double real) {
        return subtract(fromReal(real));
    }

    /**
     * Multiplies two complex numbers.
     *
     * @param c A complex number that will be multiplied with the current complex number.
     * @return A new complex number equal to the product of this complex number and the given complex number.
     */
    public ComplexNumber multiply(@NotNull ComplexNumber c) {
        final double re = Re();
        final double im = Im();
        final double cRe = c.Re();
        final double cIm = c.Im();
        return new ComplexNumber(re*cRe - im*cIm, re*cIm + im*cRe);
    }

    /**
     * Multiplies a complex number by a real number.
     *
     * @param val A real number that will be multiplied with the current complex number.
     * @return A new complex number equal to the product of this complex number and the given real number.
     */
    public ComplexNumber multiply(double val) {
        return new ComplexNumber(vector.multiply(val));
    }

    /**
     * Divides two complex numbers.
     *
     * @param c The complex number that will be used to divide the current complex number.
     * @throws ArithmeticException Throws this exception if you try and divide by 0.
     * @return A new complex number equal to this complex number divided by the given complex number.
     */
    public ComplexNumber divide(@NotNull ComplexNumber c) {
        ExceptionChecker.assertFalse(c.isZero(), new ArithmeticException("Divide by zero error."));
        final double rSquared = getRSquared();
        final double re = Re();
        final double im = Im();
        final double cRe = c.Re();
        final double cIm = c.Im();
        return new ComplexNumber((re*cRe + im*cIm) / rSquared, (im*cRe - re*cIm)/ rSquared);
    }

    /**
     * Divides a complex number by a real number.
     *
     * @param val The real number that will be used to divide the current complex number.
     * @throws ArithmeticException Throws this exception if you try to divide by 0.
     * @return A new complex number equal to this complex number divided by the given real number.
     */
    public ComplexNumber divide(double val) {
        ExceptionChecker.assertNotEqual(val, 0, new ArithmeticException("Divide by zero error."));
        return multiply(1/val);
    }

    /**
     * Raises this complex number to a complex power.
     *
     * @param c The complex number exponent that this complex number will be raised to.
     * @return A new complex number equal to this complex number raised to the power of the given complex number.
     */
    public ComplexNumber pow(ComplexNumber c) {
        return ComplexNumber.pow(this, c);
    }

    /**
     * Raises this complex number to a real power.
     *
     * @param n The real power that this complex number will be raised to.
     * @return A new complex number equal to this complex number raised to the nth power.
     */
    public ComplexNumber pow(double n) {
        return ComplexNumber.pow(this, n);
    }

    /**
     * Finds the principal nth root of this complex number.
     *
     * @param n An integer denoting which root is being taken. 2 is square root, 3 is cube root, etc.
     * @throws ArithmeticException Throws this exception if you try and pass in 0 for n.
     * @return The principal nth root of this complex number.
     */
    public ComplexNumber nthRoot(int n) {
        ExceptionChecker.assertTrue(n != 0, new ArithmeticException("Cannot take the 0th root."));
        return pow(1.0/n);
    }

    /**
     * Finds all nth roots of this complex number.
     *
     * @param n An integer denoting which root is being taken. 2 is square root, 3 is cube root, etc.
     * @return All nth roots of this complex number.
     */
    public ComplexNumber[] nthRootFull(int n) {
        final ComplexNumber firstRoot = nthRoot(n);
        final ComplexNumber[] roots = new ComplexNumber[Math.abs(n)];
        roots[0] = firstRoot;

        final double thetaStep = 2*PI/n;

        for (int i = 1; i < Math.abs(n); i++) {
            final double theta = firstRoot.getTheta();
            ComplexNumber root = firstRoot.copy();
            root.setTheta(theta + i*thetaStep);
            roots[i] = root;
        }

        return roots;
    }

    /**
     * Takes the logarithm of this complex number.
     *
     * @param base The base of the logarithm
     * @return The logarithm of this complex number.
     */
    public ComplexNumber log(double base) {
        return ComplexNumber.log(base, this);
    }

    /**
     * Takes the natural logarithm of this complex number.
     *
     * @return The natural logarithm of this complex number.
     */
    public ComplexNumber ln() {
        return ComplexNumber.ln(this);
    }

    /**
     * Converts a real number to a complex number.
     *
     * @param real A real number.
     * @return The complex number representation of the given real number.
     */
    @Contract("_ -> new")
    public static @NotNull ComplexNumber fromReal(double real) {
        return new ComplexNumber(real, 0);
    }

    /**
     * Converts an imaginary number to a complex number.
     *
     * @param imaginary The coefficient of an imaginary number.
     * @return The complex number representation of the given imaginary number.
     */
    @Contract("_ -> new")
    public static @NotNull ComplexNumber fromImaginary(double imaginary) {
        return new ComplexNumber(0, imaginary);
    }

    /**
     * Takes the logarithm of a complex number.
     *
     * @param base The base of the logarithm.
     * @param c The complex number we are taking the logarithm of.
     * @throws ArithmeticException Throws this exception if the logarithm base is <= 1.
     * @throws ArithmeticException Throws this exception if the real part of the complex number we are taking the log of is zero.
     * @return The logarithm of the given complex number.
     */
    public static ComplexNumber log(double base, @NotNull ComplexNumber c) {
        ExceptionChecker.assertTrue(base > 1, new ArithmeticException("Logarithm must have a base greater than 1."));
        ExceptionChecker.assertTrue(c.Re() != 0, new ArithmeticException("Cannot take the logarithm of a number with real part 0."));
        return ln(c).divide(Math.log(base));
    }

    /**
     * Takes the natural logarithm of a complex number.
     *
     * @param c The complex number of are taking the natural logarithm of.
     * @throws ArithmeticException Throws this exception if the real part of the complex number we are taking the log of is zero.
     * @return The natural logarithm of the given complex number.
     */
    @Contract("_ -> new")
    public static @NotNull ComplexNumber ln(@NotNull ComplexNumber c) {
        ExceptionChecker.assertTrue(c.Re() != 0, new ArithmeticException("Cannot take the logarithm of a number with real part 0."));
        return new ComplexNumber(0.5*Math.log(c.getRSquared()), c.getTheta());
    }

    /**
     * Raises a complex number to the power of another complex number.
     *
     * @param base The complex base.
     * @param exponent The complex exponent.
     * @throws ArithmeticException Throws this exception if you try and raise 0 to a complex number with real part <= 0.
     * @return Base to the power of exponent.
     */
    public static ComplexNumber pow(ComplexNumber base, ComplexNumber exponent) {
        if(base.isZero() && exponent.isZero()) return fromReal(1);
        ExceptionChecker.assertFalse(base.isZero() && base.Re() <= 0, new ArithmeticException("Cannot raise 0 to any power with real part less than or equal to 0 (except 0)."));
        if(base.isZero()) return fromReal(0);

        final double lnR = Math.log(base.getR());
        final double theta = base.getTheta();
        return exp(exponent.multiply(lnR).add(exponent.multiply(new ComplexNumber(0, theta))));
    }

    /**
     * Raises a real number to the power of a complex number.
     *
     * @param base The real base.
     * @param exponent The complex exponent.
     * @return Base to the power of exponent.
     */
    public static ComplexNumber pow(double base, ComplexNumber exponent) {
        return pow(fromReal(base), exponent);
    }

    /**
     *
     * @param base
     * @param exponent
     * @return
     */
    public static ComplexNumber pow(@NotNull ComplexNumber base, double exponent) {
        ExceptionChecker.assertTrue(base.isZero() && exponent >= 0, new ArithmeticException("Cannot raise 0 to a negative power."));
        if(base.isZero() && exponent != 0) return fromReal(0);
        else if(base.isZero()) return fromReal(1);

        return base.pow(exponent);
    }

    public static @NotNull ComplexNumber exp(@NotNull ComplexNumber c) {
        final double expA = Math.exp(c.Re());
        return new ComplexNumber(expA*cos(c.Im()), expA*sin(c.Im()));
    }

    public static ComplexNumber safeSqrt(@NotNull ComplexNumber value, int decimalPlaces) {
        ExceptionChecker.assertTrue(decimalPlaces >= 0, new ArithmeticException("Cannot have a negative number of decimal places."));
        return value.nthRoot(2).round(decimalPlaces);
    }

    public static ComplexNumber safeSqrt(ComplexNumber value) {
        return safeSqrt(value, 9);
    }

    public static ComplexNumber[] safeSqrtFull(@NotNull ComplexNumber value, int decimalPlaces) {
        ExceptionChecker.assertTrue(decimalPlaces >= 0, new ArithmeticException("Cannot have a negative number of decimal places."));
        final ComplexNumber[] roots = value.nthRootFull(2);
        for (int i = 0; i < roots.length; i++) {
            roots[i] = roots[i].round(decimalPlaces);
        }
        return roots;
    }

    public static ComplexNumber[] safeSqrtFull(ComplexNumber value) {
        return safeSqrtFull(value, 9);
    }

    public static ComplexNumber safeSqrt(double value, int decimalPlaces) {
        ExceptionChecker.assertTrue(decimalPlaces >= 0, new ArithmeticException("Cannot have a negative number of decimal places."));
        return fromReal(value).nthRoot(2).round(decimalPlaces);
    }

    public static ComplexNumber safeSqrt(double value) {
        return safeSqrt(value, 9);
    }

    public static ComplexNumber[] safeSqrtFull(double value, int decimalPlaces) {
        return safeSqrtFull(fromReal(value), decimalPlaces);
    }

    public static ComplexNumber[] safeSqrtFull(double value) {
        return safeSqrtFull(value, 9);
    }

    @Override
    public String toString() {
        final double re = Re();
        final double im = Im();
        if(im < 0) {
            return re + " - " + Math.abs(im) + "i";
        }
        return re + " + " + im + "i";
    }
}
