package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

interface Action {
    void run();
}

public class GamepadState {
    private final Gamepad gamepad;

    private final HashMap<String, Boolean> buttonsPressed = new HashMap<>();
    private final HashMap<String, Boolean> triggersPressed = new HashMap<>();

    double leftPower = 0;
    double rightPower= 0;

    private void initButtons() {
        buttonsPressed.put("a", false);
        buttonsPressed.put("b", false);
        buttonsPressed.put("x", false);
        buttonsPressed.put("y", false);
    }

    private void initTriggers() {
        triggersPressed.put("left", false);
        triggersPressed.put("right", false);
    }

    public GamepadState(Gamepad pad) {
        gamepad = pad;
        initButtons();
        initTriggers();
    }

    public double calcLeftPower(double speedRate) {
        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        leftPower = Range.clip(speedRate * (drive + 2*turn), -1.0, 1.0);
        return leftPower;
    }

    public double calcRightPower(double speedRate) {
        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        rightPower = Range.clip(speedRate * (drive - 2*turn), -1.0, 1.0);
        return rightPower;
    }

    public float getLeftStickY() {
        return gamepad.left_stick_y;
    }

    public float getRightStickX() {
        return gamepad.right_stick_x;
    }

    public double getTurn() {
        return gamepad.left_stick_x;
    }

    public void pressTrigger(String name) {
        triggersPressed.put(name, true);
    }
    public void releasedTrigger(String name) {
        triggersPressed.put(name, false);
    }

    public void pressButton(String name) {
        buttonsPressed.put(name, true);
    }

    public void releaseButton(String name) {
        buttonsPressed.put(name, false);
    }

    public void onTriggerOnce(String tr, Action cb) {
        float trigger = getTrigger(tr);
        boolean alreadyPressed = triggersPressed.get(tr);
        if (trigger > 0 && !alreadyPressed) {
            cb.run();
            pressTrigger(tr);
        } else if (trigger == 0 && alreadyPressed) {
            releasedTrigger(tr);
        }
    }

    public void onTrigger(String tr, Action cb) {
        float trigger = getTrigger(tr);
        if (trigger > 0) {
            cb.run();
            pressTrigger(tr);
        } else {
            releasedTrigger(tr);
        }
    }

    public float getTrigger(String name) {
        switch (name) {
            case "left":
                return gamepad.left_trigger;
            case "right":
                return gamepad.right_trigger;
            default:
                return 0; // not pressed so no value
        }
    }

    public void onButtonOnce(String btn, Action cb) {
        boolean button = getButton(btn);
        boolean alreadyPressed = buttonsPressed.get(btn);
        if (button && !alreadyPressed) {
            cb.run();
            pressButton(btn);
        } else if (!button && alreadyPressed) {
            releaseButton(btn);
        }
    }

    public void onButton(String btn, Action cb) {
        boolean button = getButton(btn);
        if (button) {
            cb.run();
            pressButton(btn);
        } else {
            releaseButton(btn);
        }
    }

    private boolean getButton(String name) {
        switch (name) {
            case "a":
                return gamepad.a;
            case "b":
                return gamepad.b;
            case "x":
                return gamepad.x;
            case "y":
                return gamepad.y;
            default:
                return false;
        }
    }
}
