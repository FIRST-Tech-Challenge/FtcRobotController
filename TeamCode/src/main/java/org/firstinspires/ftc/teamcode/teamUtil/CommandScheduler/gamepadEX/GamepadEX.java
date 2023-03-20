package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Trigger;

public class GamepadEX {

    public final ButtonEX
            a,
            b,
            x,
            y,

            dpad_up,
            dpad_down,
            dpad_left,
            dpad_right,

            left_bumper,
            right_bumper,
    
            left_stick_button,
            right_stick_button;

    public ContinuousInput
            leftX,
            leftY,
            rightX,
            rightY,

            left_trigger,
            right_trigger;

    Gamepad gamepad;

    public GamepadEX(Gamepad gamepad){
        this.gamepad = gamepad;

        a = new ButtonEX(() -> gamepad.a);
        b = new ButtonEX(() -> gamepad.b);
        x = new ButtonEX(() -> gamepad.x);
        y = new ButtonEX(() -> gamepad.y);

        dpad_up = new ButtonEX(() -> gamepad.dpad_up);
        dpad_down = new ButtonEX(() -> gamepad.dpad_down);
        dpad_left = new ButtonEX(() -> gamepad.dpad_left);
        dpad_right = new ButtonEX(() -> gamepad.dpad_right);

        left_bumper = new ButtonEX(() -> gamepad.left_bumper);
        right_bumper = new ButtonEX(() -> gamepad.right_bumper);
    
        left_stick_button = new ButtonEX(() -> gamepad.left_stick_button);
        right_stick_button = new ButtonEX(() -> gamepad.right_stick_button);
    
        leftX = new ContinuousInput(() -> gamepad.left_stick_x);
        leftY = new ContinuousInput(() -> -gamepad.left_stick_y);
        rightX = new ContinuousInput(() -> gamepad.right_stick_x);
        rightY = new ContinuousInput(() -> -gamepad.right_stick_y);

        left_trigger = new ContinuousInput(() -> gamepad.left_trigger);
        right_trigger = new ContinuousInput(() -> gamepad.right_trigger);
    }

    public void endLoopUpdate(){
        a.endLoopUpdate();
        b.endLoopUpdate();
        x.endLoopUpdate();
        y.endLoopUpdate();
        dpad_up.endLoopUpdate();
        dpad_down.endLoopUpdate();
        dpad_left.endLoopUpdate();
        dpad_right.endLoopUpdate();
        left_bumper.endLoopUpdate();
        right_bumper.endLoopUpdate();
    }
}

