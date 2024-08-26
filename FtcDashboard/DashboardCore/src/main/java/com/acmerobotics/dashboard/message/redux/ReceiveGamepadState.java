package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class ReceiveGamepadState extends Message {
    public static class Gamepad {
        public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
        public boolean dpad_up, dpad_down, dpad_left, dpad_right;
        public boolean a, b, x, y;
        public boolean guide, start, back;
        public boolean left_bumper, right_bumper;
        public boolean left_stick_button, right_stick_button;
        public float left_trigger, right_trigger;
        public boolean touchpad;
    }

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public ReceiveGamepadState() {
        super(MessageType.RECEIVE_GAMEPAD_STATE);
    }

    public Gamepad getGamepad1() {
        return gamepad1;
    }

    public Gamepad getGamepad2() {
        return gamepad2;
    }
}
