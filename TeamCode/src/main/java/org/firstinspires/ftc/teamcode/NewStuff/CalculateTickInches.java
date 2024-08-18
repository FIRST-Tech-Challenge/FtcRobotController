package org.firstinspires.ftc.teamcode.NewStuff;

public class CalculateTickInches {

    double ticksPerRevolution = 537.7;
    double wheelCircumfMm = 96 * Math.PI;
    double ticksPerMm = ticksPerRevolution / wheelCircumfMm;

    double mmPerInch = 25.4;

    public double mmToTicksDriveTrain(double mm) {
        return (mm * ticksPerMm);
    }

    public double inchToTicksDriveTrain(double inches) {
        return (mmToTicksDriveTrain(inches * mmPerInch));
    }
}
