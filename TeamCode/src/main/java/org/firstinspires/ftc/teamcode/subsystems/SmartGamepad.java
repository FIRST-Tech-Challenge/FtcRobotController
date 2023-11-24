package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class SmartGamepad extends Gamepad implements Subsystem {

    private Gamepad original = null;  // reference to the original gamepad object from opmode
    private Gamepad previous = null;  // a copy of gamepad with previous state

    public SmartGamepad(Gamepad gamepad) {
        original = gamepad;
        previous = new Gamepad();
        //Log.v("gamepad", "Smart gamepad check in.");
    }

    @Override
    public void update (TelemetryPacket packet) {
        previous.copy(this);
        copy(original);
    }

    public boolean dpad_up_pressed() {
        return dpad_up && !previous.dpad_up;
    }

    public boolean dpad_down_pressed() {
        return dpad_down && !previous.dpad_down;
    }

    public boolean dpad_left_pressed() {
        return dpad_left && !previous.dpad_left;
    }

    public boolean dpad_right_pressed() {
        return dpad_right && !previous.dpad_right;
    }

    public boolean left_trigger_pressed() {
        return left_trigger>0.1 && !(previous.left_trigger>0.1);
    }
    public boolean right_trigger_pressed() {
        return right_trigger>0.1 && !(previous.right_trigger>0.1);
    }

    public boolean a_pressed() {
        return a && !previous.a;
    }

    public boolean b_pressed() {
        return b && !previous.b;
    }

    public boolean x_pressed() {
        return x && !previous.x;
    }

    public boolean y_pressed() {
        return y && !previous.y;
    }
    public boolean dpad_changed() {
        //Log.v("gamepad", String.format("Dpad (up/down/left/right): %b,%b,%b,%b, previous: %b,%b,%b,%b", dpad_up, dpad_down, dpad_left, dpad_right,
        //        previous.dpad_up, previous.dpad_down, previous.dpad_left, previous.dpad_right));

        boolean result =  (dpad_up ^ previous.dpad_up)     || (dpad_down ^ previous.dpad_down) ||
                (dpad_left ^ previous.dpad_left) || (dpad_right ^ previous.dpad_right);
        if (result) {
            //Log.v("gamepad/transition", "dpad changed: True");
        }
        return result;
    }

    public boolean right_stick_released() {
        boolean result =  (Math.abs(right_stick_x) + Math.abs(right_stick_y) < 0.001) &&
                (Math.abs(previous.right_stick_x) + Math.abs(previous.right_stick_y) > 0.001);
        if (result) {
            //Log.v("gamepad/transition", "right stick released: True");
        }
        return result;
    }

    public boolean leftJoystickButton() {
        return left_stick_button && !previous.left_stick_button;
    }
}
