package org.firstinspires.ftc.teamcode.Enhancement.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Enhancement.Robot;

/**
 * GamePadConfig has the Config for the gamepad, which is used in robot.java.
 *
 * <p>It also maps the gamepad in mapGamePadInputs(), this is also used by the robot subsystem.
 * This improves Robot.java's code quality</p>
 *
 * @see Robot
 */
public class GamePadConfig extends Config {
    // Declare game pad objects
    public double leftStickX, leftStickY, rightStickX, rightStickY;
    public float triggerLeft, triggerRight;
    public boolean aButton, bButton, xButton, yButton, dPadUp, dPadDown, dPadLeft, dPadRight,
            bumperLeft, bumperRight = false;

    public double leftStickX2, leftStickY2, rightStickX2, rightStickY2;
    public float triggerLeft2, triggerRight2;
    public boolean aButton2, bButton2, xButton2, yButton2, dPadUp2, dPadDown2, dPadLeft2,
            dPadRight2, bumperLeft2, bumperRight2, isaButtonPressedPrev, isbButtonPressedPrev,
            isxButtonPressedPrev, isyButtonPressedPrev, isdPadUpPressedPrev, isdPadDownPressedPrev,
            isdPadLeftPressedPrev, isdPadRightPressedPrev, islBumperPressedPrev,
            isrBumperPressedPrev, isaButton2PressedPrev, isbButton2PressedPrev,
            isxButton2PressedPrev, isyButton2PressedPrev, isdPadUp2PressedPrev,
            isdPadDown2PressedPrev, isdPadLeft2PressedPrev, isdPadRight2PressedPrev,
            islBumper2PressedPrev, isrBumper2PressedPrev = false;

    /**
     * This maps the game pad inputs to variables, simplifying Robot.java's code and improving code quality
     *
     * @param robot the robot subsystem
     * @see Robot#getGamePadInputs()
     */
    public void mapGamePadInputs(Robot robot) {
        LinearOpMode opMode = robot.getOpMode();
        Gamepad gamePad1 = opMode.gamepad1;
        Gamepad gamePad2 = opMode.gamepad2;

        // button mapping for joystick #1
        isaButtonPressedPrev = aButton;
        isbButtonPressedPrev = bButton;
        isxButtonPressedPrev = xButton;
        isyButtonPressedPrev = yButton;
        isdPadUpPressedPrev = dPadUp;
        isdPadDownPressedPrev = dPadDown;
        isdPadLeftPressedPrev = dPadLeft;
        isdPadRightPressedPrev = dPadRight;
        islBumperPressedPrev = bumperLeft;
        isrBumperPressedPrev = bumperRight;
        leftStickX = robot.joystickDeadZoneCorrection(gamePad1.left_stick_x);
        leftStickY = robot.joystickDeadZoneCorrection(gamePad1.left_stick_y);
        rightStickX = robot.joystickDeadZoneCorrection(gamePad1.right_stick_x);
        rightStickY = robot.joystickDeadZoneCorrection(gamePad1.right_stick_y);
        triggerLeft = gamePad1.left_trigger;
        triggerRight = gamePad1.right_trigger;
        aButton = gamePad1.a;
        bButton = gamePad1.b;
        xButton = gamePad1.x;
        yButton = gamePad1.y;
        dPadUp = gamePad1.dpad_up;
        dPadDown = gamePad1.dpad_down;
        dPadLeft = gamePad1.dpad_left;
        dPadRight = gamePad1.dpad_right;
        bumperLeft = gamePad1.left_bumper;
        bumperRight = gamePad1.right_bumper;

        // button mapping for joystick #2
        isaButton2PressedPrev = aButton2;
        isbButton2PressedPrev = bButton2;
        isxButton2PressedPrev = xButton2;
        isyButton2PressedPrev = yButton2;
        isdPadUp2PressedPrev = dPadUp2;
        isdPadDown2PressedPrev = dPadDown2;
        isdPadLeft2PressedPrev = dPadLeft2;
        isdPadRight2PressedPrev = dPadRight2;
        islBumper2PressedPrev = bumperLeft2;
        isrBumper2PressedPrev = bumperRight2;
        leftStickX2 = robot.joystickDeadZoneCorrection(gamePad2.left_stick_x);
        leftStickY2 = robot.joystickDeadZoneCorrection(gamePad2.left_stick_y);
        rightStickX2 = robot.joystickDeadZoneCorrection(gamePad2.right_stick_x);
        rightStickY2 = robot.joystickDeadZoneCorrection(gamePad2.right_stick_y);
        triggerLeft2 = gamePad2.left_trigger;
        triggerRight2 = gamePad2.right_trigger;
        aButton2 = gamePad2.a;
        bButton2 = gamePad2.b;
        xButton2 = gamePad2.x;
        yButton2 = gamePad2.y;
        dPadUp2 = gamePad2.dpad_up;
        dPadDown2 = gamePad2.dpad_down;
        dPadLeft2 = gamePad2.dpad_left;
        dPadRight2 = gamePad2.dpad_right;
        bumperLeft2 = gamePad2.left_bumper;
        bumperRight2 = gamePad2.right_bumper;
    }

    @Override
    public Object get(String key) {
        return null;
    }

    @Override
    public void set(String key, Object value) {

    }
}
