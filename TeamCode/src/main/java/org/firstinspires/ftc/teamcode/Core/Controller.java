package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Controller {

    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;



    public ButtonReader y1;
    public ButtonReader x1;
    public ButtonReader a1;
    public ButtonReader b1;
    public ButtonReader b2;
    public ButtonReader a2;
    public ButtonReader x2;
    public ButtonReader y2;
    public ButtonReader gamePad1LBumper;
    public ButtonReader gamePad1RBumper;
    public ButtonReader gamePad2LBumper;
    public ButtonReader gamePad2RBumper;
    public ButtonReader gamePad1DpadUp;
    public ButtonReader gamePad1DpadDown;
    public ButtonReader gamePad1DpadRight;
    public ButtonReader gamePad1DpadLeft;
    public ButtonReader gamePad1RightStickButton;
    public ButtonReader gamePad1LeftStickButton;
    public ButtonReader gamePad2RightStickButton;
    public ButtonReader gamePad2LeftStickButton;
    public double gamepad1X;
    public double gamepad1Y;
    public double gamepad1Rot;

    public ButtonReader gamePad2DpadLeft;
    public ButtonReader gamePad2DpadRight;
    public ButtonReader gamePad2DpadDown;
    public ButtonReader gamePad2DpadUp;
    public double gamepad2Rot;
    public double gamepad2X;
    public double gamepad2Y;
    public double gamePad1RTrigger;
    public double gamePad1LTrigger;

    public double gamePad2RTrigger;
    public double gamePad2LTrigger;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Gamepad 1
        gamepad1X = gamepadEx1.getLeftX();
        gamepad1Y = gamepadEx1.getLeftY();
        gamepad1Rot = gamepadEx1.getRightX();

        a1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.A);
        b1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.B);
        x1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.X);
        y1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.Y);

        gamePad1LBumper = new ButtonReader(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER);
        gamePad1RBumper = new ButtonReader(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER);

        gamePad1DpadUp = new ButtonReader(gamepadEx1, GamepadKeys.Button.DPAD_UP);
        gamePad1DpadDown = new ButtonReader(gamepadEx1, GamepadKeys.Button.DPAD_DOWN);
        gamePad1DpadLeft = new ButtonReader(gamepadEx1, GamepadKeys.Button.DPAD_LEFT);
        gamePad1DpadRight = new ButtonReader(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT);

        gamePad1RightStickButton = new ButtonReader(gamepadEx1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad1LeftStickButton = new ButtonReader(gamepadEx1, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad1LTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad1RTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        //Gamepad 2
        gamepad2X = gamepadEx2.getLeftX();
        gamepad2Y = gamepadEx2.getLeftY();
        gamepad2Rot = gamepadEx2.getRightX();

        a2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.A);
        b2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.B);
        x2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.X);
        y2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.Y);

        gamePad2LBumper = new ButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        gamePad2RBumper = new ButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);

        gamePad2DpadUp = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_UP);
        gamePad2DpadDown = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_DOWN);
        gamePad2DpadLeft = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_LEFT);
        gamePad2DpadRight = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_RIGHT);

        gamePad2RightStickButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        gamePad2LeftStickButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_STICK_BUTTON);

        gamePad2LTrigger = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        gamePad2RTrigger = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }


}