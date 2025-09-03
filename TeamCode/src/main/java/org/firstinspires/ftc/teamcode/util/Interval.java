package org.firstinspires.ftc.teamcode.util;



// Interval class is used to represent a range with an associated output value
public class Interval {
    // Starting position of the interval
    private double startPos;

    // Ending position of the interval
    private double endPos;

    // Output value associated with this interval
    private double output;

    // Constructor to initialize the interval with start, end, and output values
    public Interval(double start, double end, double output) {
        this.startPos = start;   // Set the starting position
        this.endPos = end;       // Set the ending position
        this.output = output;    // Set the output value
    }

    // Getter method to return the starting position of the interval
    public double getStartPos() {
        return startPos;
    }

    // Getter method to return the ending position of the interval
    public double getEndPos() {
        return endPos;
    }

    // Getter method to return the output value of the interval
    public double getOutput() {
        return output;
    }
}
