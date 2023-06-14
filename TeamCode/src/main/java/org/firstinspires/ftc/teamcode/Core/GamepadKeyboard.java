package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class GamepadKeyboard {
    public Gamepad gamepad;

    public ArrayList<String> activeBefore;
    public ArrayList<String> activeAfter;
    public ArrayList<String> pressed;
    public GamepadKeyboard(Gamepad gamepad) {
        this.gamepad = gamepad;
        activeBefore = new ArrayList<String>();
        activeAfter = new ArrayList<String>();
        pressed = new ArrayList<String>();
    }

    public void update() {
        updateFinish();
        updateStart();
    }

    public void updateStart() {
        updateBefore();
        updateAfter();
    }

    public void updateFinish() {
        updateFinal();
    }

    private void updateBefore() {
        if (gamepad.left_bumper && !pressed.contains("left_bumper")) {
            activeBefore.add("left_bumper");
            pressed.add("left_bumper");
        }
        if (gamepad.dpad_up && !pressed.contains("dpad_up")) {
            activeBefore.add("dpad_up");
            pressed.add("dpad_up");
        }
        if (gamepad.dpad_left && !pressed.contains("dpad_left")) {
            activeBefore.add("dpad_left");
            pressed.add("dpad_left");
        }
        if (gamepad.dpad_down && !pressed.contains("dpad_down")) {
            activeBefore.add("dpad_down");
            pressed.add("dpad_down");
        }
        if (gamepad.dpad_right && !pressed.contains("dpad_right")) {
            activeBefore.add("dpad_right");
            pressed.add("dpad_right");
        }
        if (gamepad.y && !pressed.contains("y")) {
            activeBefore.add("y");
            pressed.add("y");
        }
        if (gamepad.x && !pressed.contains("x")) {
            activeBefore.add("x");
            pressed.add("x");
        }
        if (gamepad.a && !pressed.contains("a")) {
            activeBefore.add("a");
            pressed.add("a");
        }
        if (gamepad.b && !pressed.contains("b")) {
            activeBefore.add("b");
            pressed.add("b");
        }
        if (gamepad.right_bumper && !pressed.contains("right_bumper")) {
            activeBefore.add("right_bumper");
            pressed.add("right_bumper");
        }

        if (gamepad.left_stick_button && !pressed.contains("left_stick_button")) {
            activeBefore.add("left_stick_button");
            pressed.add("left_stick_button");
        }
        if (gamepad.right_stick_button && !pressed.contains("right_stick_button")) {
            activeBefore.add("right_stick_button");
            pressed.add("right_stick_button");
        }

        if ((gamepad.left_trigger > 0.9) && !pressed.contains("left_trigger")) {
            activeBefore.add("left_trigger");
            pressed.add("left_trigger");
        }
        if ((gamepad.right_trigger > 0.9) && !pressed.contains("right_trigger")) {
            activeBefore.add("right_trigger");
            pressed.add("right_trigger");
        }
    }
    private void updateAfter() {
        if (!gamepad.left_bumper && pressed.contains("left_bumper")) {
            activeAfter.add("left_bumper");
            pressed.remove("left_bumper");
        }
        if (!gamepad.dpad_up && pressed.contains("dpad_up")) {
            activeAfter.add("dpad_up");
            pressed.remove("dpad_up");
        }
        if (!gamepad.dpad_left && pressed.contains("dpad_left")) {
            activeAfter.add("dpad_left");
            pressed.remove("dpad_left");
        }
        if (!gamepad.dpad_down && pressed.contains("dpad_down")) {
            activeAfter.add("dpad_down");
            pressed.remove("dpad_down");
        }
        if (!gamepad.dpad_right && pressed.contains("dpad_right")) {
            activeAfter.add("dpad_right");
            pressed.remove("dpad_right");
        }
        if (!gamepad.y && pressed.contains("y")) {
            activeAfter.add("y");
            pressed.remove("y");
        }
        if (!gamepad.x && pressed.contains("x")) {
            activeAfter.add("x");
            pressed.remove("x");
        }
        if (!gamepad.a && pressed.contains("a")) {
            activeAfter.add("a");
            pressed.remove("a");
        }
        if (!gamepad.b && pressed.contains("b")) {
            activeAfter.add("b");
            pressed.remove("b");
        }
        if (!gamepad.right_bumper && pressed.contains("right_bumper")) {
            activeAfter.add("right_bumper");
            pressed.remove("right_bumper");
        }

        if (!gamepad.left_stick_button && pressed.contains("left_stick_button")) {
            activeAfter.add("left_stick_button");
            pressed.remove("left_stick_button");
        }
        if (!gamepad.right_stick_button && pressed.contains("right_stick_button")) {
            activeAfter.add("right_stick_button");
            pressed.remove("right_stick_button");
        }

        if (!(gamepad.left_trigger > 0.9) && pressed.contains("left_trigger")) {
            activeAfter.add("left_trigger");
            pressed.remove("left_trigger");
        }
        if (!(gamepad.right_trigger > 0.9) && pressed.contains("right_trigger")) {
            activeAfter.add("right_trigger");
            pressed.remove("right_trigger");
        }
    }

    private void updateFinal() {
        activeBefore.clear();
        activeAfter.clear();
    }

    public void reset() {
        pressed.clear();
        activeBefore.clear();
        activeAfter.clear();
    }
}
