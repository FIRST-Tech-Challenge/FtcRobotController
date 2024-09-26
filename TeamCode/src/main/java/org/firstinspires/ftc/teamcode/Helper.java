package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwaremaps.DeepHardwareMap;


/**
 * A helper class that provides commonly used functions, primarily for OpModes
 */
public class Helper {
    /**
     * Enum for storing a colour's state
     */
    public enum GamepadColour {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255);

        private final int red;
        private final int green;
        private final int blue;

        /**
         * Set up colour
         * @param r Red Value
         * @param g Green Value
         * @param b Blue Value
         */
        GamepadColour(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }
    }

    /**
     * Switch for if Gamepad should rumble on certain actions.
     * IE. Colour switch
     */
    public static boolean GamepadRumble = true;

    /**
     * Logs operational state of all motors
     * @param hardwareMap The DeepHardwareMap being used
     * @param telemetry Telemetry object from OpMode for logging
     */
    public static void ReportDriveMotorStatus(DeepHardwareMap hardwareMap, Telemetry telemetry) {
        telemetry.addData("Front Right Motor", hardwareMap.FrontRightMotor == null ? "Fault" : "Operational");
        telemetry.addData("Front Left Motor", hardwareMap.FrontLeftMotor == null ? "Fault" : "Operational");
        telemetry.addData("Back Right Motor", hardwareMap.BackRightMotor == null ? "Fault" : "Operational");
        telemetry.addData("Back Left Motor", hardwareMap.BackLeftMotor == null ? "Fault" : "Operational");
    }

    /**
     * Logs current power of all values provided
     * @param powers Array of powers of motors. Commonly returned from Mecanum.Move()
     * @param telemetry Telemetry object from OpMode for logging
     */
    public static void ReportMecanumMotorSpeed(double[] powers, Telemetry telemetry) {
        telemetry.addData("Front Right Motor Power", powers[0]);
        telemetry.addData("Front Left Motor Power", powers[1]);
        telemetry.addData("Back Right Motor Power", powers[2]);
        telemetry.addData("Back Left Motor Power", powers[3]);
    }

    /**
     * Logs current power of all drive motors in hardware map
     * @param hardwareMap DeepHardwareMap currently in use
     * @param telemetry Telemetry object from OpMode for logging
     */
    public static void ReportMecanumMotorSpeed(DeepHardwareMap hardwareMap, Telemetry telemetry) {
        telemetry.addData("Front Right Motor Power", hardwareMap.FrontRightMotor.getPower());
        telemetry.addData("Front Left Motor Power", hardwareMap.FrontLeftMotor.getPower());
        telemetry.addData("Back Right Motor Power", hardwareMap.BackRightMotor.getPower());
        telemetry.addData("Back Left Motor Power", hardwareMap.BackLeftMotor.getPower());
    }

    /**
     * Sets light on gamepad and rumbles
     * @param gp Gamepad to modify colour
     * @param col Colour to change gamepad to
     */
    public static void SetGamepadLight(Gamepad gp, GamepadColour col) {
        if(GamepadRumble) gp.rumble(200);
        gp.setLedColor(col.red, col.green, col.blue, Gamepad.LED_DURATION_CONTINUOUS);
    }

    /**
     * Gets all of the RHS buttons from the gamepad in order of ABXY
     * @param gp The gamepad to get the state from
     * @return Array of boolean values signifying each buttons' state
     */
    public static boolean[] CopyButtonsFromGamepad(Gamepad gp) {
        return new boolean[] {gp.a, gp.b, gp.x, gp.y};
    }
}
