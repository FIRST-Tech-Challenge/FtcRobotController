package org.firstinspires.ftc.teamcode.ultimategoal2020;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;

import java.util.ArrayList;

import static java.lang.String.format;

public class PoseError2020 {
    //  PoseError represents the difference in position and heading
    //  between a given pose and a target pose
    //  By default these are in FIELD coordinate system
    //  But may need to check the positionErrorVector coordinate system to verify
    //  This class provides a method to transform the error into the ROBOT's coordinate system

    /***************************************************************
    ******    CLASS VARIABLES
    ***************************************************************/
    private double headingErrorDeg;     //Error in which way the robot is facing
    private FieldPosition positionError;    //Field position object for X,Y error from robot's targetPose

    private ArrayList<ErrorSum2020> errorSums2020;      //Arraylist of all error Sums (X, Y, Spin)

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
        for(ErrorSum2020 errorSum2020 : errorSums2020){
            if(errorSum2020.getCsysDirection() == dir){
                errorSumValue = errorSum2020.getValue();
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
    public PoseError2020(){
        this.headingErrorDeg = 0;
        this.positionError = new FieldPosition(0,0, CoordinateSystem.FIELD);
        initializeErrorSums();
    }

    public PoseError2020(EbotsRobot2020 robot) {
        //  Set the errorSum to zero when instantiated
        resetErrorSums();
        initializeError(robot);
    }

    /***************************************************************
    ******    CLASS INSTANCE METHODS
    ***************************************************************/

    public void resetErrorSums(){
        this.initializeErrorSums();
    }

    public void initializeError(EbotsRobot2020 robot){
        //Initialize error
        this.positionError = new FieldPosition();
        calculateError(robot,0, Speed.SLOW);
        initializeErrorSums();
    }

    public void calculateError(EbotsRobot2020 robot, long loopDuration, Speed speed){
        double xError = robot.getTargetPose().getX() - robot.getActualPose().getX();
        double yError = robot.getTargetPose().getY() - robot.getActualPose().getY();
        this.headingErrorDeg = Pose2020.applyAngleBound(robot.getTargetPose().getHeadingDeg() - robot.getActualPose().getHeadingDeg());
        this.positionError.setxPosition(xError);
        this.positionError.setyPosition(yError);

        //Now add the integrator if the loop duration is greater than 0
        if(loopDuration > 0) {
            updateErrorSums(robot, loopDuration, speed);
        }
    }

    private void initializeErrorSums(){
        errorSums2020 = new ArrayList<>();
        //Loop through each direction
        for(CsysDirection dir:CsysDirection.values()){
            //Skip Z Direction
            if(dir != CsysDirection.Z){
                //Add a new errorSum object to the list
                errorSums2020.add(new ErrorSum2020(dir));
            }
        }
    }

    private void updateErrorSums(EbotsRobot2020 robot, long loopDuration, Speed speed){
        for(ErrorSum2020 errorSum2020 : errorSums2020){
            errorSum2020.update(robot, loopDuration, speed);
        }
    }

    public FieldPosition getPositionErrorInRobotCoordinateSystem(EbotsRobot2020 robot){
        //this is used in auton to determine how the robot must drive to achieve target pose

        //Note: the positionErrorVector is the distance between robot and target pose in FIELD coordinate system
        //Step 1:  Call the Coordinate System routine to perform the rotation transformation
        FieldPosition positionErrorInRobotCoordinateSystem = CoordinateSystem.transformCoordinateSystem(this.positionError,CoordinateSystem.ROBOT,robot.getActualPose().getHeadingDeg());
        return positionErrorInRobotCoordinateSystem;
    }

    @Override
    public String toString(){
        return "xError [xErrorSum]: " + format("%.2f", this.getXError()) + " ["+ format("%.2f", this.getXErrorSum()) + "]" +
                "\n yError [yErrorSum]: " + format("%.2f", this.getYError()) + " ["+ format("%.2f", this.getYErrorSum()) + "]" +
                "\n spin Error [spinErrorSum]: " + format("%.2f", this.getHeadingErrorDeg()) + " ["+ format("%.2f", this.getHeadingErrorDegSum()) + "]";
    }

}
