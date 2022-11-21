package org.firstinspires.ftc.teamcode;
/*
class created by Stephen Duffy
do not edit
*/


//a simple class that stores an x and Y value
public class Point {
    public enum Action {NONE} //Add new actions here later on
    protected double x;
    protected double y;
    protected Action action = Action.NONE;
    protected double strafePower = 0.0;
    protected double rotatePower = 0.0;
    // Prefix with POI if it's a point of interest (in which case it is a stopping point in Auton).

    Point(double X,double Y){
        x=X;
        y=Y;
    }
    Point(double X,double Y,Action action, double strafePower, double rotatePower){
        x=X;
        y=Y;
        this.action = action;
        this.strafePower = strafePower;
        this.rotatePower = rotatePower;
    }
    void setX(double X){
        x=X;
    }
    void setY(double Y){
        y=Y;
    }
}
