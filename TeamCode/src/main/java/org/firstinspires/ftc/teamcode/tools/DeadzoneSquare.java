package org.firstinspires.ftc.teamcode.tools;

public class DeadzoneSquare {
    private double u;
    private double v;
    private double deadzone;
    private double finalPower;

    public DeadzoneSquare(double a, double b, double c){ //a is minHorizontal, b is minPower, c is maxPower
        u = (b-c) / (a*a-1);
        v = (a*a*c - b) / (a*a - 1);
        deadzone = a;
    }

    public double computePower(double joystick){
        if (Math.abs(joystick) < deadzone){
            return 0.0;
        }
        finalPower = u * Math.pow(joystick,2) + v;
        if(joystick > 0.0){
            return finalPower;
        } else{
            return -finalPower;
        }
    }
}