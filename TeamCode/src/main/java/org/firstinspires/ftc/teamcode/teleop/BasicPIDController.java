package org.firstinspires.ftc.teamcode.teleop;


//import com.acmerobotics.dashboard.config.Config;

public class BasicPIDController {

    //variable declaration 9999
    private double kP;
    private final double kI;
    private final double kD;
    private double previousError;
    private double totalError;
    private double velocityError;
    private double lastTimeStamp;

    //constructor
    public BasicPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public double calculate(int measuredPosition, int targetPosition) {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        int positionError = targetPosition - measuredPosition; //position
        totalError += ((positionError + previousError) / 2) * period; //integral
        velocityError = (positionError - previousError) / period; //derivative
        previousError = positionError;

        return positionError * kP + totalError * kI + velocityError * kD;
    }
    public void setP(double p) {
        this.kP = p;
    }
}