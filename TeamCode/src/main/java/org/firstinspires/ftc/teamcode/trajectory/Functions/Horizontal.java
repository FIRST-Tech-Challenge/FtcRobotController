package org.firstinspires.ftc.teamcode.trajectory.Functions;

public class Horizontal implements ProfileFunction {
    private final double velo;
    private final double startPos;

    public Horizontal(double velo, double startPos) {
        this.velo = velo;
        this.startPos = startPos;
    }


    @Override
    public double getPosition(double t) {
        return velo*t + startPos;
    }

    @Override
    public double getVelocity(double t) {
        return velo;
    }
}
