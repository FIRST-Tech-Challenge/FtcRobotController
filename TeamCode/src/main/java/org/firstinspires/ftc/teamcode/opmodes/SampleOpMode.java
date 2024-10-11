package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

/***
 * Sample OpMode that uses the CommandOpMode base class.
 *
 */
public class SampleOpMode extends CommandOpMode {
    // GamepadEx is a better version of the gamepad object you are used to.
    // You can setup command to be ran on button press, release, or hold
    // you also don't have to invert the value from the triggers and sticks. 
    private GamepadEx driver;
    private GamepadEx operator;


    @Override
    public void initialize() {
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }
}
