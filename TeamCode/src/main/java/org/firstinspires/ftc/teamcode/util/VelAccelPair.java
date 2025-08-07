package org.firstinspires.ftc.teamcode.util;

public class VelAccelPair {
    private double velocity;
    private double acceleration;

    public VelAccelPair(double velocity, double acceleration){
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getVelocity(){
        return velocity;
    }

    public double getAcceleration(){
        return acceleration;
    }

    public void setVelocity(double velocity){
        this.velocity = velocity;
    }

    public void setAcceleration(double acceleration){
        this.acceleration = acceleration;
    }
}
