package org.firstinspires.ftc.teamcode.auto.Math;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

public class SplineMath {
    Constants constants = new Constants();
    SpinPID spinPIDR;
    SpinPID spinPIDL;
    private int RinitClick;
    private int LinitClick;
    private double x;
    private double y;
    private double turnAmount; //amount robot header should turn (for table-spinning)

    public SplineMath(){
        spinPIDR = new SpinPID();
        spinPIDL = new SpinPID();
    }

    public void setInits(int initialClickR, int initialClickL){
        RinitClick = initialClickR;
        LinitClick = initialClickL;
    }

    public void setPos(double x, double y, double theta){
        this.x = x;
        this.y = y;
        turnAmount = theta;
        spinPIDL.setTargets(returnDistance()[1], 0.1, 0, 0);
        spinPIDR.setTargets(returnDistance()[2], 0.1, 0, 0);
    }

    public double[] returnDistance(){
        double[] distanceArr = new double[3];
        double radius = ((x * x) + (y * y)) / (2 * x);
        double theta = 0;

        if (x==0){ //linear movement if x=0. No splining
            distanceArr[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            distanceArr[1] = distanceArr[0]; //left distance
            distanceArr[2] = distanceArr[0]; //right distance
            return distanceArr;
        } //this has to be fixed, because robots can move in vertical lines even when x!=0 (for auto specifically)

        try{
            radius = ((x * x) + (y * y)) / (2 * x);
            theta = Math.acos(
                    1 - (
                            ((x * x) + (y * y)) / (2 * radius * radius)
                    )
            );
        } catch (ArithmeticException e){
//            System.out.println("Error: " + e);
        }

        distanceArr[0] = radius * theta; //center distance
        distanceArr[1] = (radius + constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) * theta; //left distance
        distanceArr[2] = (radius - constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) * theta; //right distance
        return distanceArr;
    }

    public int[] getClicks(){
        int[] clicks = new int[4];
        int spinClicksR = (int)(returnDistance()[2] * constants.CLICKS_PER_INCH);
        int spinClicksL = (int)(returnDistance()[1] * constants.CLICKS_PER_INCH);
        int rotationClicks = (int)(turnAmount * constants.CLICKS_PER_DEGREE); //table spinning clicks

        clicks[0] = spinClicksL + rotationClicks; //left side
        clicks[1] = -spinClicksL + rotationClicks; //left side

        clicks[2] = spinClicksR + rotationClicks; //right side
        clicks[3] = -spinClicksR + rotationClicks; //right side
        return clicks;
    }

    public double returnPowerR(double currentClickR){
        double distanceR = returnDistance()[2];
        double distanceL = returnDistance()[1];

        double distance = Math.abs(currentClickR - RinitClick) * constants.CLICKS_PER_INCH;
        return (distanceR >= distanceL ? 1 * spinPIDR.update(distance): (distanceL / distanceR) * spinPIDR.update(distance));
    }

    public double returnPowerL(double currentClickL){
        double distanceR = returnDistance()[2];
        double distanceL = returnDistance()[1];

        double distance = Math.abs(currentClickL - LinitClick) * constants.CLICKS_PER_INCH;
        return (distanceL >= distanceR ? 1 * spinPIDL.update(distance) : (distanceR / distanceL) * spinPIDL.update(distance));
    }
}
