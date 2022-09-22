package org.firstinspires.ftc.teamcode.ebotsutil;

import static java.lang.String.format;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

import java.util.ArrayList;

public class PoseError {
    //  PoseError represents the difference in position and heading
    //  between a given pose and a target pose
    //  By default these are in FIELD coordinate system
    //  But may need to check the positionErrorVector coordinate system to verify
    //  This class provides a method to transform the error into the ROBOT's coordinate system

    /***************************************************************
    ******    INSTANCE VARIABLES
    ***************************************************************/
    private double headingErrorDeg;     //Error in which way the robot is facing
    private FieldPosition positionError;    //Field position object for X,Y error from robot's targetPose

    private ArrayList<ErrorSum> errorSums;      //Arraylist of all error Sums (X, Y, Spin)

    private EbotsAutonOpMode opMode;

    private String logTag = "EBOTS";

    /***************************************************************
     ******    SIMPLE GETTERS AND SETTERS
     ***************************************************************/

    public double getXError() {return this.getErrorComponent(CsysDirection.X);}
    public double getYError() {return this.getErrorComponent(CsysDirection.Y);}

    public double getErrorComponent(CsysDirection dir){
        double errorValue = 0;

        if(dir == CsysDirection.X | dir == CsysDirection.Y) {
            errorValue = positionError.getPositionComponent(dir);
        }
        else if(dir == CsysDirection.Heading) {
            errorValue = headingErrorDeg;
        }
        return errorValue;
    }

    public double getHeadingErrorDeg() {return headingErrorDeg;}
    public double getHeadingErrorRad() {return Math.toRadians(headingErrorDeg);}

    public double getXErrorSum() {return getErrorSumComponent(CsysDirection.X);}
    public double getYErrorSum() {return getErrorSumComponent(CsysDirection.Y);}
    public double getHeadingErrorDegSum() {return getErrorSumComponent(CsysDirection.Heading);}

    public double getErrorSumComponent(CsysDirection dir){
        double errorSumValue = 0;
        for(ErrorSum errorSum: errorSums){
            if(errorSum.getCsysDirection() == dir){
                errorSumValue = errorSum.getValue();
                break;
            }
        }
        return errorSumValue;
    }
    /***************************************************************
     ******    CALCULATED PROPERTIES
     ***************************************************************/
    public double getMagnitude() {return this.positionError.getXYMagnitude(); }
    public double getFieldErrorDirectionDeg() {return this.positionError.getFieldErrorDirectionDeg();}

    /***************************************************************
    ******    CONSTRUCTORS
    ***************************************************************/
    public PoseError(){
        this.headingErrorDeg = 0;
        this.positionError = new FieldPosition(0,0, CoordinateSystem.FIELD);
        initializeErrorSums();
    }

    public PoseError(Pose currentPose, Pose targetPose, EbotsAutonOpMode opMode) {
        this.opMode = opMode;
        //  Set the errorSum to zero when instantiated
        resetErrorSums();
        initializeError(currentPose, targetPose);
    }

    /***************************************************************
    ******    INSTANCE METHODS
    ***************************************************************/

    public void resetErrorSums(){
        this.initializeErrorSums();
    }

    public void initializeError(Pose currentPose, Pose targetPose){
        //Initialize error
        this.positionError = new FieldPosition();
        calculateError(currentPose, targetPose,0);
        initializeErrorSums();
    }

    public void calculateError(Pose currentPose, Pose targetPose, long loopDuration){
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        this.headingErrorDeg = UtilFuncs.applyAngleBounds(targetPose.getHeadingDeg() - currentPose.getHeadingDeg());
        this.positionError.setxPosition(xError);
        this.positionError.setyPosition(yError);

        // Now add the integrator if the loop duration is greater than 0
        if(loopDuration > 0) {
            updateErrorSums(loopDuration);
        }
    }

    private void initializeErrorSums(){
        errorSums = new ArrayList<>();
        //Loop through each direction
        for(CsysDirection dir:CsysDirection.values()){
            //Skip Z Direction
            if(dir != CsysDirection.Z){
                //Add a new errorSum object to the list
                errorSums.add(new ErrorSum(dir));
            }
        }
    }

    private void updateErrorSums(long loopDuration){
        boolean debugOn = false;

        Speed speed = this.opMode.getMotionController().getSpeed();
        if(debugOn){
            Log.d(logTag, "Updating error sums, speed: " + speed.name());
        }
        for(ErrorSum errorSum:errorSums){
            CsysDirection dir = errorSum.getCsysDirection();
            boolean isIntegratorOn = speed.isIntegratorOn(dir);
            if (debugOn) Log.d(logTag, "for direction: " + dir.name() + " integratorOn: " + isIntegratorOn);
            // if integrator is off for current direction then skip the rest of the loop and move to next one
            if (! isIntegratorOn) continue;

            boolean isSignalSaturated = opMode.getMotionController().isSignalSaturated(dir);
            double currentError = this.getErrorComponent(dir);

            // see if error is same sign as errorSum.  If errorSum is 0 (initial value), then considered same sign
            boolean sameSign = Math.signum(currentError) == Math.signum(errorSum.getValue());
            sameSign = sameSign | errorSum.getValue() == 0.0;

            // only update the errorSum for a given direction if either
            // --> the signal is NOT saturated
            // --> the error and saturated signal are opposite sign
            if(!isSignalSaturated  | !sameSign){
                // loop duration is in ms, initially this code multiplied loopDuration * 1000
                // this should probaly be divided by 1000 so the unit is
                errorSum.update(loopDuration, currentError);
            }
        }
    }


    @Override
    public String toString(){
        return "xError [xErrorSum]: " + format("%.2f", this.getXError()) + " ["+ format("%.2f", this.getXErrorSum()) + "]" +
                "\n yError [yErrorSum]: " + format("%.2f", this.getYError()) + " ["+ format("%.2f", this.getYErrorSum()) + "]" +
                "\n spin Error [spinErrorSum]: " + format("%.2f", this.getHeadingErrorDeg()) + " ["+ format("%.2f", this.getHeadingErrorDegSum()) + "]";
    }

}
