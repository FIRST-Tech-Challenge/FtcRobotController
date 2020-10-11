package org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation;

import java.io.Serializable;
import java.util.Comparator;

public class RobotPoint2D implements Serializable {
    private static final long serialVersionUID = 1L;
    public static class XComparator implements Comparator<RobotPoint2D>{
        @Override
        public int compare(RobotPoint2D o1, RobotPoint2D o2) {
            if(o1.X > o2.X){
                return 1;
            }else if(o1.X == o2.X){
                return 0;
            }else{
                return -1;
            }
            //return o1.X - o2.X;
        }
    }
    public static class YComparator implements Comparator<RobotPoint2D>{

        @Override
        public int compare(RobotPoint2D o1, RobotPoint2D o2) {
            if(o1.Y > o2.Y){
                return 1;
            }else if(o1.Y == o2.Y){
                return 0;
            }else{
                return -1;
            }
            //return o1.Y - o2.Y;
        }
    }
    public double X;
    public double Y;
    public RobotPoint2D(double X, double Y) {
    	this.X = X;
    	this.Y = Y;
    }
    public RobotPoint2D(RobotPoint2D oldPoint) {
    	this.X = oldPoint.X;
    	this.Y = oldPoint.Y;
    }
    public RobotPoint2D(RobotVector2D indicator) {
    	this.X = indicator.X;
    	this.Y = indicator.Y;
    }
    public double distanceTo(RobotPoint2D secondPoint){
        return Math.hypot(secondPoint.X - this.X,secondPoint.Y - this.Y);
    }
    public double distanceToOrigin(){
        return Math.hypot(this.X,this.Y);
    }
    public void setValues(RobotPoint2D value){
        this.X = value.X;
        this.Y = value.Y;
    }

    public boolean equals(RobotPoint2D point){
        if(this.X == point.X && this.Y == point.Y){
            return true;
        }else{
            return false;
        }
    }
    @Override
    public String toString(){
        return "[" + this.X + ", " + this.Y + "]";
    }
}
