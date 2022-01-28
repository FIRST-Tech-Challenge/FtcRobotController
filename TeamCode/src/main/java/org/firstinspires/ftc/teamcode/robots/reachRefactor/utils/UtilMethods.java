package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.EPSILON;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

public class UtilMethods {

    public static double servoNormalize(double pulse) {
        return (pulse - 750.0) / 1500.0; // convert mr servo controller pulse width to double on 0 - 1 scale
    }
    public static double wrapAngle(double angle){
        return ((angle % 360) + 360) % 360;
    }
    public static double wrapAnglePlus(double angle1, double angle2){
        return (((angle1 + angle2) % 360) + 360) % 360;
    }
    public static double wrapAngleMinus(double angle1, double angle2){
        return (((angle1 - angle2) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public static double wrapAngleRad(double angle){
        return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }
    public static double wrapAnglePlusRad(double angle1, double angle2){
        return (((angle1 + angle2) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }
    public static double wrapAngleMinusRad(double angle1, double angle2){
        return (((angle1 - angle2) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public static boolean notDeadZone(double value, double threshold){
        if (value> -threshold && value < threshold) return false;
        else return true;
    }

    public static double nextCardinal(double currentAngle, boolean right, double hop){
        double tmp;
        if (right) {
            tmp = (currentAngle + hop) % 360;
            tmp = Math.floor(tmp/90) + 1;
            if (tmp>=4) tmp = 0;
        }
        else{
            tmp = (currentAngle - hop);
            tmp = Math.floor(tmp/90);
            if (tmp<0) tmp = 3;
        }
        return tmp*90;
    }

    public static StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }

    public static double deadZone(double x, double threshold) {
        return Math.abs(x) < threshold ? 0 : x;
    }

    public static boolean approxEquals(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }

    public static double max(double... values) {
        double max = Double.NEGATIVE_INFINITY;
        for(double value : values)
            if(value > max)
                max = value;
        return max;
    }

    public static int max(int... values) {
        int max = Integer.MIN_VALUE;
        for(int value : values)
            if(value > max)
                max = value;
        return max;
    }

    public static int servoClip(int position) {
        return Range.clip(position, 750, 2250);
    }
}
