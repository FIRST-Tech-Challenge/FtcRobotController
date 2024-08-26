package org.firstinspires.ftc.teamcode.Controllers;

public class FeedForward {
    public double kV;
    public double kA;
    public double kS;
    public FeedForward(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }
    public double calculate(double desiredVelocity, double desiredAcceleration){
        return kV*desiredVelocity+kA*desiredAcceleration+kS*Math.signum(desiredVelocity);
    }
}


