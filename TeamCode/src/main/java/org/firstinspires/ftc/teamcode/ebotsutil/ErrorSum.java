package org.firstinspires.ftc.teamcode.ebotsutil;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;

public class ErrorSum {
    /**
     * Error sum represents the cumulative error value used in the integrator control
     * of the PID controller.  Each coordinate (X, Y, Heading) has its own ErrorSum
     * and a PoseError object contains an array list of these objects.
     */
    private double value;
    private CsysDirection csysDirection;
//    private CoordinateSystem coordinateSystem = CoordinateSystem.FIELD;

    public ErrorSum(CsysDirection dir){
        this.value = 0;
        this.csysDirection = dir;
    }

    public double getValue() {
        return value;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public void update(long loopDurationMillis, double currentError){
        //  Step 1:  See if integrator must be updated
        //           Add the integrator if the following conditions are met
        //              1) Loop Duration > 0s AND



        //Set the value to zero if duration is 0 or integrator is off
        if (loopDurationMillis <= 0) {
            this.value = 0.0;
            return;
        }

        double deltaT = loopDurationMillis / 1000.0;  //convert to seconds
        this.value += (currentError * deltaT);  //Get area under error vs. time curve (in * s)
    }

}
