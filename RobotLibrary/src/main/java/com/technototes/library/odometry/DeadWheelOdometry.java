package com.technototes.library.odometry;

import com.technototes.library.util.DoubleDifferential;

import java.util.function.DoubleSupplier;

public class DeadWheelOdometry implements Odometry {

    private DoubleSupplier leftPod;
    private DoubleSupplier rightPod;
    private DoubleSupplier backPod;

    private double x, y;

    private DoubleDifferential deltaL, deltaR, deltaB;

    private DoubleDifferential deltaF, deltaA, deltaS;

    public DeadWheelOdometry(DoubleSupplier l, DoubleSupplier r, DoubleSupplier b){
        new DeadWheelOdometry(l, r, b, 0, 0, 0);

    }
    public DeadWheelOdometry(DoubleSupplier l, DoubleSupplier r, DoubleSupplier b, double startx, double starty, double starta){
        l = leftPod;
        r = rightPod;
        b = backPod;
        x=startx;
        y=starty;
        deltaF = new DoubleDifferential();
        deltaA = new DoubleDifferential(starta);
        deltaS = new DoubleDifferential();
        deltaL = new DoubleDifferential();
        deltaR = new DoubleDifferential();
        deltaB = new DoubleDifferential();
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getRotation() {
        return deltaA.getCurrValue();
    }

    @Override
    public DeadWheelOdometry update() {
        deltaL.update(getLeftPod().getAsDouble());
        deltaR.update(getRightPod().getAsDouble());
        deltaB.update(getBackPod().getAsDouble());

        deltaF.update((deltaL.getDeltaV()+deltaR.getDeltaV())/2);
        deltaA.update(((deltaL.getDeltaV()-deltaR.getDeltaV())*6/*constant*/)/2);
        deltaS.update(deltaB.getDeltaV()-deltaA.getDeltaV()*(4/6)/*constant*/);

        double avgAngle = deltaA.getCurrValue()-deltaA.getDeltaV()/2;

        x+=Math.sin(avgAngle)*deltaF.getDeltaV()+Math.cos(avgAngle)*deltaS.getDeltaV();
        y+=Math.cos(avgAngle)*deltaF.getDeltaV()-Math.sin(avgAngle)*deltaS.getDeltaV();

        return this;
    }

    public DoubleSupplier getLeftPod() {
        return leftPod;
    }

    public DoubleSupplier getRightPod() {
        return rightPod;
    }

    public DoubleSupplier getBackPod() {
        return backPod;
    }

}
