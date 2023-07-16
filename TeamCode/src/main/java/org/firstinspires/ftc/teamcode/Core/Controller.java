package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final GamepadKeyboard gamepadKeyboard1;
    private final GamepadKeyboard gamepadKeyboard2;


    //public boolean dumperToggle;
    public boolean gamepad1StrafeToggle;
    public boolean gamepad2StrafeToggle;
    public boolean gamepad1RotationToggle;
    public boolean gamepad2RotationToggle;
    public boolean y1;
    public boolean x1;
    public boolean a1;
    public boolean b1;
    public boolean b2;
    public boolean a2;
    public boolean x2;
    public boolean y2;
    public boolean gamePad1LBumper;
    public boolean gamePad1Back;
    public boolean gamePad1RBumper;
    public boolean gamePad1DpadUp;
    public boolean gamePad1DpadDown;
    public boolean gamePad1DpadRight;
    public boolean gamePad1DpadLeft;
    public boolean gamePad1RightStickButton;
    public boolean gamePad1leftStickButton;
    public double gamepad1X;
    public double gamepad1Y;
    public double gamepad1Rot;
    public double gamepad2X;
    public double gamepad2Y;
    public boolean gamepad2DpadLeft;
    public boolean gamepad2DpadRight;
    public boolean gamepad2DpadDown;
    public boolean gamepad2DpadUp;
    public double gamepad2Rot;
    public boolean gamePad1RTrigger;
    public boolean gamePad1LTrigger;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.gamepadKeyboard1 = new GamepadKeyboard(gamepad1);
        this.gamepadKeyboard2 = new GamepadKeyboard(gamepad2);
    }

    public void update() {
        gamepadKeyboard1.update();
        gamepadKeyboard2.update();

        //Button Names - DO NOT CHANGE______________________________________________________________

        //GamePad1
        gamepad1X = gamepad1.left_stick_x;
        gamepad1Y = -gamepad1.left_stick_y;
        gamepad1Rot = gamepad1.right_stick_x;
        gamepad1StrafeToggle = gamepadKeyboard1.activeBefore.contains("left_bumper");
        gamepad1RotationToggle = gamepadKeyboard1.activeBefore.contains("right_bumper");

        gamePad1DpadDown = gamepadKeyboard1.activeBefore.contains("dpad_down");
        gamePad1DpadUp = gamepadKeyboard1.activeBefore.contains("dpad_up");
        gamePad1DpadRight = gamepadKeyboard1.activeBefore.contains("dpad_right");
        gamePad1DpadLeft = gamepadKeyboard1.activeBefore.contains("dpad_left");

        b1 = gamepadKeyboard1.activeBefore.contains("b");
        a1 = gamepadKeyboard1.activeBefore.contains("a");
        x1 = gamepadKeyboard1.activeBefore.contains("x");
        y1 = gamepadKeyboard1.activeBefore.contains("y");

        gamePad1RBumper = gamepad1.right_bumper;
        gamePad1LBumper = gamepad1.left_bumper;

        gamePad1RTrigger = gamepadKeyboard1.activeBefore.contains("right_trigger");
        gamePad1LTrigger = gamepadKeyboard1.activeBefore.contains("left_trigger");

        gamePad1Back = gamepad1.back;

        gamePad1RightStickButton =  gamepadKeyboard1.activeBefore.contains("right_stick_button");
        gamePad1leftStickButton = gamepadKeyboard1.activeBefore.contains("left_stick_button");

        //GamePad2
        gamepad2X = gamepad2.left_stick_x;
        gamepad2Y = -gamepad2.left_stick_y;
        gamepad2Rot = gamepad2.right_stick_x;
        gamepad2StrafeToggle = gamepadKeyboard2.activeBefore.contains("left_bumper");
        gamepad2RotationToggle = gamepadKeyboard2.activeBefore.contains("right_bumper");

        gamepad2DpadLeft = gamepad2.dpad_left;
        gamepad2DpadRight = gamepad2.dpad_right;
        gamepad2DpadDown = gamepad2.dpad_down;
        gamepad2DpadUp = gamepad2.dpad_up;

        b2 = gamepadKeyboard2.activeBefore.contains("b");
        a2 = gamepadKeyboard2.activeBefore.contains("a");
        x2 = gamepadKeyboard2.activeBefore.contains("x");
        y2 = gamepadKeyboard2.activeBefore.contains("y");
    }
}