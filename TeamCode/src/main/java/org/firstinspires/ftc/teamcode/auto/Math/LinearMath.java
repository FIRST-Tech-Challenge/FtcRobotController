package org.firstinspires.ftc.teamcode.auto.Math;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class LinearMath { //Note: snap() is used in the auto class separately. This class is used assuming that the wheels are already pointing the way we want it to.
    Constants constants = new Constants();
    SpinPID spinPID;

    private double initialX;
    private double initialY;

    private double x;
    private double y;
    private double theta; //amount robot header should turn (for table-spinning)


    public LinearMath(){
        spinPID = new SpinPID();
    }

    public void setInits(double initXClicks, double initYClicks){
        initialX = initXClicks * constants.INCHES_PER_CLICK;
        initialY = initYClicks * constants.INCHES_PER_CLICK;
    }

    public void setPos(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
        spinPID.setTargets(getDistance(), 0.1, 0, 0);
    }

    private double getDistance(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        double translationClicks = getDistance() * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = theta * constants.CLICKS_PER_DEGREE; //table spinning clicks

        clicks[0] = (int)(translationClicks + rotationClicks);
        clicks[1] = (int)(-translationClicks + rotationClicks);
        clicks[2] = (int)(translationClicks + rotationClicks);
        clicks[3] = (int)(-translationClicks + rotationClicks);
        return clicks;
    }

    public double getSpinPower(double currentXClicks, double currentYClicks){
        currentXClicks *= constants.INCHES_PER_CLICK;
        currentYClicks *= constants.INCHES_PER_CLICK;
        double distanceTravelled = Math.sqrt(Math.pow(currentXClicks - initialX, 2) + Math.pow(currentYClicks - initialY, 2));

        return spinPID.update(distanceTravelled);
    }
}
