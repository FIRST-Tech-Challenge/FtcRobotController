package com.kalipsorobotics.math;

public class CalculateTickInches {

    double ticksPerRevolution = 2000;
    double odoCircumfMm = 48 * Math.PI;
    double ticksPerMm = ticksPerRevolution / odoCircumfMm;

    double mmPerInch = 25.4;

    public double mmToTicksDriveTrain(double mm) {
        return (mm * ticksPerMm);
    }

    public double ticksToMmDriveTrain(double ticks) {
        return (ticks / ticksPerMm);
    }

    public double inchToTicksDriveTrain(double inches) {
        return (mmToTicksDriveTrain(inches * mmPerInch));
    }

    public double ticksToInchesDriveTrain(double ticks) {
        return (ticksToMmDriveTrain(ticks / mmPerInch));
    }
}
