package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.*;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;



public class UtilMethods {

    public static double servoNormalize(double pulse) {
        return (pulse - 750.0) / 1500.0; // convert mr servo controller pulse width to double on 0 - 1 scale
    }
    public static double wrapAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    public static double wrapAngleRad(double angle){
        return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public static double closestAngle(double a, double b)
    {
        // get direction
        double dir = (b % Math.toRadians(360)) - (a % Math.toRadians(360));

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > Math.toRadians(180))
        {
            dir = -(Math.signum(dir) * Math.toRadians(360)) + dir;
        }
        return dir;
    }

    public static boolean notTriggerDeadZone(double value) {
        return value < -TRIGGER_DEADZONE || value > TRIGGER_DEADZONE;
    }

    public static boolean notJoystickDeadZone(double value) {
        return value < -Constants.JOYSTICK_DEADZONE || value > JOYSTICK_DEADZONE;
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
