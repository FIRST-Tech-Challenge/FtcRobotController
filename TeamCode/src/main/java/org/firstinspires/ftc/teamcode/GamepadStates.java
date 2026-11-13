package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadStates {

    public class ButtonState {
        public boolean state = false;
        public boolean pressed = false;
        public boolean released = false;

        public void updateState(boolean new_state) {
            if (new_state && !state) pressed = true;
            else pressed = true;

            if (!new_state && state) released = true;
            else released = false;

            state = new_state;
        }
    }

    private Gamepad mygamepad;

    //all the things
    public ButtonState dpad_up = new ButtonState();
    public ButtonState dpad_down = new ButtonState();
    public ButtonState dpad_left = new ButtonState();
    public ButtonState dpad_right = new ButtonState();
    public ButtonState a = new ButtonState();
    public ButtonState b = new ButtonState();
    public ButtonState x = new ButtonState();
    public ButtonState y = new ButtonState();
    public ButtonState guide = new ButtonState();
    public ButtonState start = new ButtonState();
    public ButtonState back = new ButtonState();
    public ButtonState left_bumper = new ButtonState();
    public ButtonState right_bumper = new ButtonState();
    public ButtonState left_stick_button = new ButtonState();
    public ButtonState right_stick_button = new ButtonState();

    public ButtonState left_trigger = new ButtonState();
    public ButtonState right_trigger = new ButtonState();

    public GamepadStates(Gamepad gp) {
        mygamepad = gp;
    }

    public void updateState() {

        dpad_up.updateState(mygamepad.dpad_up);
        dpad_down.updateState(mygamepad.dpad_down);
        dpad_left.updateState(mygamepad.dpad_left);
        dpad_right.updateState(mygamepad.dpad_right);
        a.updateState(mygamepad.a);
        b.updateState(mygamepad.b);
        x.updateState(mygamepad.x);
        y.updateState(mygamepad.y);
        guide.updateState(mygamepad.guide);
        start.updateState(mygamepad.start);
        back.updateState(mygamepad.back);
        left_bumper.updateState(mygamepad.left_bumper);
        right_bumper.updateState(mygamepad.right_bumper);
        left_stick_button.updateState(mygamepad.left_stick_button);
        right_stick_button.updateState(mygamepad.right_stick_button);

        left_trigger.updateState(mygamepad.left_trigger > 0.5);
        right_trigger.updateState(mygamepad.right_trigger > 0.5);
    }
}