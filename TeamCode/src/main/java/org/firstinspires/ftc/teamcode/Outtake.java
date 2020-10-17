package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.acos;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

// (x, y) end effector coordinates
// a -> linkage one, rad from pos x axis
// b -> linkage two
public class Outtake {
    private final int BAR_LENGTH = 120;
    private final int BAR_OFFSET = 76;
    private final int STEP_INTERVAL = 20;
    private final int PICKUP_OFFSET = -125;
    private final int DEFAULT_EXTENSION = 200;

    public double[] processMovement(double x_current, double y_current, double x_input, double y_input){
        x_input *= STEP_INTERVAL;
        y_input *= STEP_INTERVAL;

        double x = x_current + x_input;
        double y = y_current + y_input;

        if(x < 50 && y_input < 0) //so it doesn't hit the robot
            y_input = 0;
        else if((Math.hypot(x - BAR_OFFSET, y - 0) > BAR_LENGTH * 2 - 15    //so it doesn't overextend left or right pivots
                || Math.hypot(x + BAR_OFFSET, y - 0) > BAR_LENGTH * 2 - 15)
                && y_input > 0)
            y_input = 0;

        if(Math.hypot(x - 177, y - 64) < 120 && x_input > 0)    //so it doesn't hit right hard stop
            x_input = 0;
        else if(Math.hypot(x + 177, y - 64) < 120 && x_input < 0)      //so it doesn't hit left hard stop
            x_input = 0;

        return getReverseKinematics(x_current + x_input, y_current + y_input, true);
    }

    //return servo positions at position
    public double[] getDeployServoPos(double y) {
        //search through coordinates for higher y position
        y = Math.ceil((y - PICKUP_OFFSET)/STEP_INTERVAL);
        return getReverseKinematics(0, y, false);
    }

    public double[] getRetractServoPos(double y) {
        //search through coordinates for lower y position
        y = Math.floor((y - PICKUP_OFFSET)/STEP_INTERVAL);
        return getReverseKinematics(0, y, false);
    }


    public double[] getReverseKinematics(double x, double y, boolean state){
        double a, b;

        int stateSign = state ? 1: -1;
        double diagonal = atan2(y,x-BAR_OFFSET);
        diagonal = (diagonal + 2*PI) % (2*PI); //creating positive 0-360 range

        double offset = stateSign*acos((sqrt(pow(x-BAR_OFFSET,2) + pow(y,2))/(2*BAR_LENGTH)));

        a = diagonal - offset;
        b = diagonal + offset;

        return new double[]{transposeServoPos(a), transposeServoPos(b)};
    }

    //takes in a 0-360 range and converts to 0-1 value. Hardware dependent.
    public double transposeServoPos(double angle){
        return (angle - 70)/270;
    }

    public int getSTEP_INTERVAL(){ return STEP_INTERVAL; }

    public int getDEFAULT_EXTENSION(){ return DEFAULT_EXTENSION; }

    public int getPICKUP_OFFSET(){ return PICKUP_OFFSET; }
}
