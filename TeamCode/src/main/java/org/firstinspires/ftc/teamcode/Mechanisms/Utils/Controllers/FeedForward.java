package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Controllers;

public class FeedForward {
    public double kV;
    public double kA;
    public double kS;

    /***
     *
     * @param kV (double) Voltage constant
     * @param kA (double) Acceleration constant
     * @param kS (double) Static constant
     */
    public FeedForward(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    /***
     *
     * @param desiredVelocity (double) The target velocity of our bot
     * @param desiredAcceleration (double) The target acceleration of our bot
     * @return (double) The voltage calculation to send to the motors
     */
    public double calculate(double desiredVelocity, double desiredAcceleration){
        return kV*desiredVelocity+kA*desiredAcceleration+kS*Math.signum(desiredVelocity);
    }
    public void setGains(double kV, double kA, double kS){
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }
}

