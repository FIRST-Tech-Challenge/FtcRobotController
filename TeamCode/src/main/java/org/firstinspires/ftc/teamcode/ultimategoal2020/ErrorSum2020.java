package org.firstinspires.ftc.teamcode.ultimategoal2020;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;

public class ErrorSum2020 {
    /**
     * Error sum represents the cumulative error value used in the integrator control
     * of the PID controller.  Each coordinate (X, Y, Heading) has its own ErrorSum
     * and a PoseError object contains an array list of these objects.
     */
    private double value;
    private CsysDirection csysDirection;
    private CoordinateSystem coordinateSystem;

    public ErrorSum2020(CsysDirection dir){
        this.value = 0;
        this.csysDirection = dir;
        this.coordinateSystem = CoordinateSystem.FIELD;
    }

    public double getValue() {
        return value;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public void update(EbotsRobot2020 robot, long loopDuration, Speed speed){
        //  Step 1:  See if integrator must be updated
        //           Add the integrator if the following conditions are met
        //              1) Loop Duration > 0s AND
        //              2) Either:
        //                  a) raw signal not saturated or
        //                  b) error is different sign than error sum

        //Check if the integrator is active for the current direction
        // (X or Y look at translate Integrator coefficient k_i, heading looks at Spin s_i)
        boolean isIntegratorOn = speed.isIntegratorOn(this.csysDirection);  //assume translate

        //Set the value to zero if duration is 0 or integrator is off
        if (loopDuration <= 0 | !isIntegratorOn) {
            this.value = 0.0;
            return;
        }

        boolean signalSaturated = robot.getDriveCommand().isSignalSaturated(this.csysDirection);

        double currentError = robot.getPoseError().getErrorComponent(this.csysDirection);
        boolean sameSign = Math.signum(currentError) == Math.signum(this.value);


        if(!signalSaturated  | !sameSign){
            // loop duration is in ms, initially this code multiplied loopDuration * 1000
            // this should probaly be divided by 1000 so the unit is
            //double deltaT = loopDuration * 1000.0;
            double deltaT = loopDuration / 1000.0;  //convert to seconds
            this.value += (currentError * deltaT);  //Get area under error vs. time curve (in * s)
        }
    }

}
