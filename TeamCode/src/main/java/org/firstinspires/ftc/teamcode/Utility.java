package org.firstinspires.ftc.teamcode;

public class Utility {

    public static double max(double[] arr){
        double result = 0;
        for (double val : arr) {
            if (Math.abs(val) > result) {
                result = Math.abs(val);
            }
        }
        return result;
    }

    public static double wrapIMU(double angle){
        if (angle >= 0){
            return angle;
        }
        else{
            return 2 * Math.PI + angle;
        }
    }

    public static double wrapIMUDeg(double angle){
        if (angle >= 0){
            return angle;
        }
        else{
            return 360 + angle;
        }
    }

    public static double unwrapDeg(double angle){
        if (angle <= 180){
            return angle;
        }
        else{
            return angle - 360;
        }
    }

    public static double clamp(double input, double upper, double lower){
        if (input <= lower){
            return lower;
        }
        else if (input >= upper){
            return upper;
        }
        else{
            return input;
        }
    }

    public static double expo(double input, double power){
        if (input <= 0){
            return -Math.pow(Math.abs(input), power);
        }
        else{
            return Math.pow(Math.abs(input), power);
        }
    }

    public static double average(double[] array){

        double sum = 0;

        for (double v : array) {
            sum += v;
        }
        return sum/array.length;
    }

    public static boolean isInRange(double input) {
        return (input > 30 && input < 200) || (input > 250 && input < 340);
    }
}
