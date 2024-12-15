package org.firstinspires.ftc.teamcode;

public class CheckDriveStraight {

    public enum DIRECTION {LEFT, RIGHT, UNKOWN}

//    public static final double TOLERANCE = 5.0;

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
    public static DIRECTION turnToCorrectSide(double angle, double target){
        if(target==180 && angle < 0) {
            System.out.println("here");
            target=-180;
        }
        if(target < angle){
            return DIRECTION.LEFT;
        } else if (target > angle){
            return DIRECTION.RIGHT;
        }
        return DIRECTION.UNKOWN;
    }

    // Normalize angles to be within the range [-180, 180]
    public static int correctionAngle(int angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

//    public static void main(String[] args) {
//        double testAngle = -174;
//        double targetAngle = 180;
////        System.out.println(correctionAngle(-180));
//        if (isWithinTolerance(testAngle, targetAngle)) {
//            System.out.println(testAngle + " is within 5 degrees of " + targetAngle);
//        } else {
//            System.out.println(testAngle + " is NOT within 5 degrees of " + targetAngle);
//            System.out.println(turnToCorrectSide(testAngle, targetAngle));
//        }
//    }
}