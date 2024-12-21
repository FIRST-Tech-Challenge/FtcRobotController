package org.firstinspires.ftc.teamcode.auto;

//where all functions for tolerance and if passed target
//basically a utils class
public class CheckDriveStraight {

    public enum DIRECTION {LEFT, RIGHT, UNKOWN}

    // Function to check if an angle is within 5 degrees of the target angle
    public static boolean isWithinTolerance(int angle, int target, int tolerance) {
        int lowerBound = (target - tolerance) ;
        int upperBound = (target + tolerance) ;

        if(lowerBound <= upperBound){
            return lowerBound <= angle && angle <= upperBound;
        } else {
            return lowerBound >= angle && angle >= upperBound;

        }
    }
    public static boolean turnToCorrectSide(double angle, double target){
        if(target==180 && angle < 0) {
            System.out.println("here");
            target=-180;
        }
        if(target < angle){
            return false;
        } else if (target > angle){
            return true;
        }
        return true;
    }
    public static boolean passedTarget(int input, int target){
        if(target > 0){
            return input >= target;
        } else {
            return input <= target;
        }
    }
}