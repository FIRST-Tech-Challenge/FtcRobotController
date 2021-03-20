package org.firstinspires.ftc.samplecode.strafer.odometry;

public class DoubleDifferential {
    private double currValue;
    private double deltaV;
    public DoubleDifferential(){
        currValue = 0;
    }
    public DoubleDifferential(double startingValue){
        currValue = startingValue;
    }
    public DoubleDifferential update(double newValue){
        deltaV = newValue-currValue;
        currValue = newValue;
        return this;
    }

    public double getCurrValue() {
        return currValue;
    }

    public double getDeltaV() {
        return deltaV;
    }
}
