package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

public class Utils {

    /**
     * convert servo controller pulse width to double on 0 - 1 scale
     * @param pulse pwm signal to be converted
     * @return
     */
    public static double servoNormalize(double pulse) {
        //todo, these numbers go back to Modern Robotics Hardare
        //todo, REV expansion hubs have a range of 500 to 2500ms
        //todo, but we can't change this unless we're ready to re-calibrate all servo settings
        return (pulse - 750.0) / 1500.0;
    }


    public static int servoToPWM(double setting){
        return (int)((setting * 2000)+500);
    }

    public static int servoClip(int position) {
        return Range.clip(position, 750, 2250);
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

    public static boolean approxEquals(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }

    public static boolean notTriggerDeadZone(double value) {
        return value < -TRIGGER_DEADZONE || value > TRIGGER_DEADZONE;
    }

    public static boolean notJoystickDeadZone(double value) {
        return value < -Constants.JOYSTICK_DEADZONE || value > JOYSTICK_DEADZONE;
    }

    public static boolean joysticksActive(Gamepad gamepad) {
        return  Utils.notJoystickDeadZone(gamepad.left_stick_x) ||
                Utils.notJoystickDeadZone(gamepad.left_stick_y) ||
                Utils.notJoystickDeadZone(gamepad.right_stick_x) ||
                Utils.notJoystickDeadZone(gamepad.right_stick_y);
    }

    public static StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }
}
