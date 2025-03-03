package org.firstinspires.ftc.teamcode.PurePursuit.Base.Controllers;

public class FullStateFeedBack implements Controller {

    private double kPos;
    private double kVel;

    public FullStateFeedBack(double kPos, double kVel) {
        this.kPos = kPos;
        this.kVel = kVel;
    }

    @Override
    public double calculate(double[] pose, double[] velocities) {
        return (kPos * (pose[0] - pose[1])) + (kVel * (velocities[0] - velocities[1]));
    }

    public void setK(double kPos, double kVel) {
        this.kPos = kPos;
        this.kVel = kVel;
    }

    public double[] getK() {
        return new double[]{kPos, kVel};
    }
}
