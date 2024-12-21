package org.firstinspires.ftc.teamcode.Hardware.Wrappers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {

    private Gamepad gamepad;

    // Controller Joystick constants | {rx, ry, lx, ly}
    public final static double[] pS4 = {1, -1, 1, -1};
    public final static double[] xBox = {-1, -1, -1, -1};
    public final static double[] logitech = {1, -1, 1, -1};
    public final static double[] rev = {1, -1, 1, -1};

    // Joystick Coefficients
    private double[] coefficients;

    // Joystick Raw & Inverted Values
    private double[] rawSticks = {0, 0, 0, 0};
    private double[] sticks = {0, 0, 0, 0};



    public Controller(Gamepad gamepad, double[] controllerType) {
        this.gamepad = gamepad;
        coefficients = controllerType;
    }

    public void setControllerType(double[] controllerType) {
        coefficients = controllerType;
    }

    public boolean getA() {
        return gamepad.a;
    }
    public boolean getB() {
        return gamepad.b;
    }
    public boolean getX() {
        return gamepad.x;
    }
    public boolean getY() {
        return gamepad.y;
    }

    public boolean getDPUp() {
        return gamepad.dpad_up;
    }
    public boolean getDPDown() {
        return gamepad.dpad_down;
    }
    public boolean getDPLeft() {
        return gamepad.dpad_left;
    }
    public boolean getDPRight() {
        return gamepad.dpad_right;
    }

    public boolean getGuide() {
        return gamepad.guide;
    }
    public boolean getBack() {
        return gamepad.back;
    }
    public boolean getStart() {
        return gamepad.start;
    }

    public boolean getLB() {
        return gamepad.left_bumper;
    }
    public boolean getRB() {
        return gamepad.right_bumper;
    }

    public boolean getLSB() {
        return gamepad.left_stick_button;
    }
    public boolean getRSB() {
        return gamepad.right_stick_button;
    }

    public double getLT() {
        return gamepad.left_trigger;
    }
    public double getRT() {
        return gamepad.right_trigger;
    }

    public double getRSX() {
        return gamepad.right_stick_x * coefficients[0];
    }
    public double getRSY() {
        return gamepad.right_stick_y * coefficients[1];
    }
    public double getLSX() {
        return gamepad.left_stick_x * coefficients[2];
    }
    public double getLSY() {
        return gamepad.left_stick_y * coefficients[3];
    }

}
