package org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils;

import static org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils.Constants.*;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StickyGamepad {

    private Gamepad gamepad;

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;
    public boolean left_trigger, right_trigger;
    public boolean left_stick_button, right_stick_button;

    private boolean dpad_up_down, dpad_down_down, dpad_left_down, dpad_right_down;
    private boolean a_down, b_down, x_down, y_down;
    private boolean left_bumper_down, right_bumper_down;
    private boolean left_trigger_down, right_trigger_down;
    private boolean left_stick_button_down, right_stick_button_down;

    public StickyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad.dpad_down) {
            if (dpad_down_down) {
                dpad_down = false;
            } else {
                dpad_down_down = true;
                dpad_down = true;
            }
        } else {
            dpad_down = false;
            dpad_down_down = false;
        }

        if (gamepad.dpad_up) {
            if (dpad_up_down) {
                dpad_up = false;
            } else {
                dpad_up_down = true;
                dpad_up = true;
            }
        } else {
            dpad_up = false;
            dpad_up_down = false;
        }

        if (gamepad.dpad_left) {
            if (dpad_left_down) {
                dpad_left = false;
            } else {
                dpad_left_down = true;
                dpad_left = true;
            }
        } else {
            dpad_left = false;
            dpad_left_down = false;
        }

        if (gamepad.dpad_right) {
            if (dpad_right_down) {
                dpad_right = false;
            } else {
                dpad_right_down = true;
                dpad_right = true;
            }
        } else {
            dpad_right = false;
            dpad_right_down = false;
        }

        if (gamepad.a) {
            if (a_down) {
                a = false;
            } else {
                a_down = true;
                a = true;
            }
        } else {
            a = false;
            a_down = false;
        }

        if (gamepad.b) {
            if (b_down) {
                b = false;
            } else {
                b_down = true;
                b = true;
            }
        } else {
            b = false;
            b_down = false;
        }

        if (gamepad.x) {
            if (x_down) {
                x = false;
            } else {
                x_down = true;
                x = true;
            }
        } else {
            x = false;
            x_down = false;
        }

        if (gamepad.y) {
            if (y_down) {
                y = false;
            } else {
                y_down = true;
                y = true;
            }
        } else {
            y = false;
            y_down = false;
        }

        if (gamepad.left_bumper) {
            if (left_bumper_down) {
                left_bumper = false;
            } else {
                left_bumper_down = true;
                left_bumper = true;
            }
        } else {
            left_bumper = false;
            left_bumper_down = false;
        }

        if (gamepad.right_bumper) {
            if (right_bumper_down) {
                right_bumper = false;
            } else {
                right_bumper_down = true;
                right_bumper = true;
            }
        } else {
            right_bumper = false;
            right_bumper_down = false;
        }

        if (MathUtil.deadZone(gamepad.left_trigger, TRIGGER_DEADZONE) > 0) {
            if (left_trigger_down) {
                left_trigger = false;
            } else {
                left_trigger_down = true;
                left_trigger = true;
            }
        } else {
            left_trigger = false;
            left_trigger_down = false;
        }

        if (MathUtil.deadZone(gamepad.right_trigger, TRIGGER_DEADZONE) > 0) {
            if (right_trigger_down) {
                right_trigger = false;
            } else {
                right_trigger_down = true;
                right_trigger = true;
            }
        } else {
            right_trigger = false;
            right_trigger_down = false;
        }

        if (gamepad.left_stick_button) {
            if (left_stick_button_down) {
                left_stick_button = false;
            } else {
                left_stick_button_down = true;
                left_stick_button = true;
            }
        } else {
            left_stick_button = false;
            left_stick_button_down = false;
        }

        if (gamepad.right_stick_button) {
            if (right_stick_button_down) {
                right_stick_button = false;
            } else {
                right_stick_button_down = true;
                right_stick_button = true;
            }
        } else {
            right_stick_button = false;
            right_stick_button_down = false;
        }
    }
}
