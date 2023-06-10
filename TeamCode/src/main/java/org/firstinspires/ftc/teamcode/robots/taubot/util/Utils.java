package org.firstinspires.ftc.teamcode.robots.taubot.util;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.ELBOW_TO_WRIST;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.EPSILON;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.JOYSTICK_DEADZONE;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.SHOULDER_TO_ELBOW;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.TRIGGER_DEADZONE;

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
        //todo, these numbers go back to Modern Robotics Hardware
        //todo, REV expansion hubs have a range of 500 to 2500ms
        //todo, but we can't change this unless we're ready to re-calibrate all servo settings
        return (pulse - 750.0) / 1500.0;
    }

    public static double servoDenormalize(double thing){
        return thing*1500+750;
    }

    /**
     * convert servo controller pulse width to double on 0 - 1 scale
     * @param pulse pwm signal to be converted
     * @return
     */
    public static double servoNormalizeExtended(double pulse) {

        //use for extended REV expansion hubs range of 500 to 2500ms
        if (pulse<500) pulse = 500;
        if (pulse>2500) pulse = 2500;
        return (pulse - 500.0) / 2000.0;
    }

    public static boolean withinErrorPercent(double value, double target, double percent){
        return (Math.abs(target-value)/target <= percent);
    }

    public static boolean withinError(double value, double target, double error){
        return (Math.abs(target-value) <= error);
    }

    public static double distanceBetweenAngles(double currentAngle, double targetAngle){
        double result = wrapAngle(targetAngle - currentAngle);
        if(result >180) return result - 360;
        return result;
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

    public static double wrapAngleMinus(double angle1, double angle2){
        //return 360-((angle1 + angle2) % 360);
        return wrapAngle(wrapAngle(angle1) - wrapAngle(angle2));
    }

    public static double wrapAngleRad(double angle){
        return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    public static double wrapAngleMinusRad(double angle){
        return 2*Math.PI - wrapAngleRad(angle);
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

    public static double map(double x, double imin, double imax, double omin, double omax) {
        return omin + (omax - omin) * ((x - imin) / (imax - imin));
    }

    public static double[] craneIK(double dx, double dy) {
        double theta2 = -Math.acos((Math.pow(dx, 2) + Math.pow(dy, 2) - Math.pow(SHOULDER_TO_ELBOW, 2) - Math.pow(ELBOW_TO_WRIST, 2)) / (2 * SHOULDER_TO_ELBOW * ELBOW_TO_WRIST));
        double theta1 = Math.atan2(dy, dx) - Math.atan2(ELBOW_TO_WRIST * Math.sin(theta2), SHOULDER_TO_ELBOW + ELBOW_TO_WRIST * Math.cos(theta2));

        if(Double.isNaN(theta1) || Double.isNaN(theta2))
            return null;

        double shoulderAngle = wrapAngle(90 - Math.toDegrees(theta1));
        double elbowAngle = 180 - wrapAngle(Math.toDegrees(-theta2));
        double wristAngle = 90 - (wrapAngle(Math.toDegrees(-theta2)) - wrapAngle(Math.toDegrees(theta1)));

        return new double[] {shoulderAngle, elbowAngle, wristAngle, wrapAngle(wristAngle + 90)};
    }

    public static double[] craneIK(double dx, double dy, double offset) {
        double theta2 = -Math.acos((Math.pow(dx, 2) + Math.pow(dy, 2) - Math.pow(SHOULDER_TO_ELBOW, 2) - Math.pow(ELBOW_TO_WRIST, 2)) / (2 * SHOULDER_TO_ELBOW * ELBOW_TO_WRIST));
        double theta1 = offset + Math.atan2(dy, dx) - Math.atan2(ELBOW_TO_WRIST * Math.sin(theta2), SHOULDER_TO_ELBOW + ELBOW_TO_WRIST * Math.cos(theta2));

        if(Double.isNaN(theta1) || Double.isNaN(theta2))
            return null;

        double shoulderAngle = wrapAngle(90 - Math.toDegrees(theta1));
        double elbowAngle = 180 - wrapAngle(Math.toDegrees(-theta2));
        double wristAngle = 90 - (wrapAngle(Math.toDegrees(-theta2)) - wrapAngle(Math.toDegrees(theta1)));

        return new double[] {shoulderAngle, elbowAngle, wristAngle, wrapAngle(wristAngle + 90)};
    }
}
