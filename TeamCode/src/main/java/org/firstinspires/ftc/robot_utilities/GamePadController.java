package org.firstinspires.ftc.robot_utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePadController {

    public Gamepad gamepad;

    private boolean pressedA, pressedB, pressedX, pressedY;
    private boolean pressedLeft, pressedRight, pressedUp, pressedDown;
    private boolean pressedRB, pressedLB;


    public GamePadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }


    public void update() {
        if(gamepad.a) {
            pressedA = true;
        }
        if(gamepad.b) {
            pressedB = true;
        }
        if(gamepad.x) {
            pressedX = true;
        }
        if(gamepad.y) {
            pressedY = true;
        }

        if(gamepad.dpad_left) {
            pressedLeft = true;
        }
        if(gamepad.dpad_right) {
            pressedRight = true;
        }
        if(gamepad.dpad_down) {
            pressedDown = true;
        }
        if(gamepad.dpad_up) {
            pressedUp = true;
        }

        if(gamepad.left_bumper) {
            pressedLB = true;
        }
        if(gamepad.right_bumper) {
            pressedRB = true;
        }
    }

    public boolean isARelease() {
        if(pressedA && !gamepad.a) {
            pressedA = false;
            return true;
        }
        return false;
    }
    public boolean isBRelease() {
        if(pressedB && !gamepad.b) {
            pressedB = false;
            return true;
        }
        return false;
    }
    public boolean isXRelease() {
        if(pressedX && !gamepad.x) {
            pressedX = false;
            return true;
        }
        return false;
    }
    public boolean isYRelease() {
        if(pressedY && !gamepad.y) {
            pressedY = false;
            return true;
        }
        return false;
    }

    public boolean isLeftRelease() {
        if(pressedLeft && !gamepad.dpad_left) {
            pressedLeft = false;
            return true;
        }
        return false;
    }
    public boolean isRightRelease() {
        if(pressedRight && !gamepad.dpad_right) {
            pressedRight = false;
            return true;
        }
        return false;
    }
    public boolean isUpRelease() {
        if(pressedUp && !gamepad.dpad_up) {
            pressedUp = false;
            return true;
        }
        return false;
    }
    public boolean isDownRelease() {
        if(pressedDown && !gamepad.dpad_down) {
            pressedDown = false;
            return true;
        }
        return false;
    }

    public boolean isLBRelease() {
        if(pressedLB && !gamepad.left_bumper) {
            pressedLB = false;
            return true;
        }
        return false;
    }
    public boolean isRBRelease() {
        if(pressedRB && !gamepad.right_bumper) {
            pressedRB = false;
            return true;
        }
        return false;
    }
}
