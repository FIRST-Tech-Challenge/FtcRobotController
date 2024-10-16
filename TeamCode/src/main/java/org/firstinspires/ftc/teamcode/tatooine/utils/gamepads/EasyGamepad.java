package org.firstinspires.ftc.teamcode.tatooine.utils.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

public class EasyGamepad {
    boolean triangleTap = false;
    boolean circleTap = false;
    boolean crossTap = false;

    boolean squareTap = false;

    boolean dPadUpTap = false;

    boolean dPadDownTap = false;

    boolean dPadRightTap = false;

    boolean dPadLeftTap = false;

    boolean rightBumperTap = false;

    boolean leftBumperTap = false;
    Gamepad gamepad;

    public EasyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean getDPadLeft() {
        if (!dPadLeftTap) {
            dPadLeftTap = true;
        } else {
            dPadLeftTap = true;
        }
        return gamepad.dpad_left;
    }

    public boolean getDPadRight() {
        if (!dPadRightTap) {
            dPadRightTap = true;
        } else {
            dPadRightTap = true;
        }
        return gamepad.dpad_right;
    }

    public boolean getDPadDown() {
        if (!dPadDownTap) {
            dPadDownTap = true;
        } else {
            dPadDownTap = true;
        }
        return gamepad.dpad_down;
    }

    public boolean getDPadUp() {
        if (!dPadUpTap) {
            dPadUpTap = true;
        } else {
            dPadUpTap = true;
        }
        return gamepad.dpad_up;
    }

    public boolean getL1() {
        if (!leftBumperTap) {
            leftBumperTap = true;
        } else {
            leftBumperTap = true;
        }
        return gamepad.left_bumper;
    }

    public boolean getR1() {
        if (!rightBumperTap) {
            rightBumperTap = true;
        } else {
            rightBumperTap = true;
        }
        return gamepad.right_bumper;
    }


    public boolean getSquare() {
        if (!squareTap) {
            squareTap = true;
        } else {
            squareTap = true;
        }
        return gamepad.square;
    }

    public boolean getTriangle() {
        if (!triangleTap) {
            triangleTap = true;
        } else {
            triangleTap = true;
        }
        return gamepad.triangle;
    }

    public boolean getCircle() {
        if (!circleTap) {
            circleTap = true;
        } else {
            circleTap = true;
        }
        return gamepad.circle;
    }

    public boolean getCross() {
        if (!crossTap) {
            crossTap = true;
        } else {
            crossTap = true;
        }
        return gamepad.cross;
    }

    public boolean wasCrossDoubleTapped() {
        getCross();
        return crossTap;
    }

    public boolean wasTriangleDoubleTapped() {
        getTriangle();
        return triangleTap;
    }

    public boolean wasSquareDoubleTapped() {
        getSquare();
        return squareTap;

    }

    public boolean wasCircleDoubleTapped() {
        getCircle();
        return circleTap;
    }

    public boolean wasDPadUpDoubleTapped() {
        getDPadUp();
        return dPadUpTap;
    }

    public boolean wasdPadDownDoubleTapped() {
        getDPadDown();
        return dPadDownTap;
    }

    public boolean wasDPadRightDoubleTapped() {
        getDPadRight();
        return dPadRightTap;
    }

    public boolean wasDPadLeftDoubleTapped() {
        getDPadLeft();
        return dPadLeftTap;
    }

    public boolean wasR1DoubleTapped() {
        getR1();
        return rightBumperTap;
    }

    public boolean wasL1DoubleTapped() {
        getL1();
        return leftBumperTap;
    }

    public void update(Gamepad gamepad) {
        this.gamepad.copy(gamepad);
    }

    public double getRightStickXPower() {
        return gamepad.right_stick_x;
    }

    public double getRightStickYPower() {
        return gamepad.right_stick_y;
    }

    public double getLeftStickXPower() {
        return gamepad.left_stick_x;
    }

    public double getLeftStickYPower() {
        return gamepad.left_stick_y;
    }

    public double getR2() {
        return gamepad.right_trigger;
    }

    public double getL2() {
        return gamepad.left_trigger;
    }
}
