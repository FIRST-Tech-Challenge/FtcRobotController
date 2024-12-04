package com.kalipsorobotics.math;

public class CalculateTickPer {

    static double ticksPerRevolutionOdometry = 2000;
    static double ticksPerRevolutionLS = 145.1;
    static double lsCircumfMm = 36 * Math.PI;
    static double odoCircumfMm = 48 * Math.PI;

    static final public double MAX_RANGE_LS_TICKS = 2050;
    static final public double MIN_RANGE_LS_TICKS = -30;


    static double mmPerInch = 25.4;

    public static double getTicksPerMm(double ticksPerRevolution, double circumf) {
        return ticksPerRevolution/circumf;
    }

    public static double mmToTicksDriveTrain(double mm) {
        return (mm * getTicksPerMm(ticksPerRevolutionOdometry, odoCircumfMm));
    }

    public static double ticksToMmDriveTrain(double ticks) {
        return (ticks / getTicksPerMm(ticksPerRevolutionOdometry, odoCircumfMm));
    }

    public static double inchToTicksDriveTrain(double inches) {
        return (mmToTicksDriveTrain(inches * mmPerInch));
    }

    public static double ticksToInchesDriveTrain(double ticks) {
        return (ticksToMmDriveTrain(ticks / mmPerInch));
    }

    public static double mmToTicksLS(double mm) {
        //single string going across so tick x2
        return 2 * (mm * getTicksPerMm(ticksPerRevolutionLS, lsCircumfMm));
    }

    public static double ticksToMmLS(double ticks) {
        return (ticks / getTicksPerMm(ticksPerRevolutionLS, lsCircumfMm));
    }

    public static double inchToTicksLS(double inches) {
        return (mmToTicksLS(inches * mmPerInch));
    }

    public static double ticksToInchesLS(double ticks) {
        return (ticksToMmLS(ticks / mmPerInch));
    }
}
