package org.firstinspires.ftc.teamcode.dragonswpilib;

import com.qualcomm.robotcore.hardware.Gamepad;

public class JoystickButton extends Button {

    public static final class XboxControllerConstants {
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kDpadUp = 5;
        public static final int kDpadDown = 6;
        public static final int kDpadLeft = 7;
        public static final int kDpadRight = 8;
        public static final int kGuide = 9;
        public static final int kStart = 10;
        public static final int kBack = 11;
        public static final int kLeftBumper = 12;
        public static final int kRightBumper = 13;
        public static final int kLeftStickButton = 14;
        public static final int kRightStickButton = 15;
        public static final int kLeftTrigger = 16;
        public static final int kRightTrigger = 17;
    }

    private final Gamepad mGamepad;
    private final int mButtonNumber;

    public JoystickButton(Gamepad gamepad, int buttonNumber) {
        mGamepad = gamepad;
        mButtonNumber = buttonNumber;
    }

    @Override
    public boolean get() {
        switch(mButtonNumber) {
            case XboxControllerConstants.kA:
                return mGamepad.a;
            case XboxControllerConstants.kB:
                return mGamepad.b;
            case XboxControllerConstants.kX:
                return mGamepad.x;
            case XboxControllerConstants.kY:
                return mGamepad.y;
            case XboxControllerConstants.kDpadUp:
                return mGamepad.dpad_up;
            case XboxControllerConstants.kDpadDown:
                return mGamepad.dpad_down;
            case XboxControllerConstants.kDpadLeft:
                return mGamepad.dpad_left;
            case XboxControllerConstants.kDpadRight:
                return mGamepad.dpad_right;
            case XboxControllerConstants.kGuide:
                return mGamepad.guide;
            case XboxControllerConstants.kStart:
                return mGamepad.start;
            case XboxControllerConstants.kBack:
                return mGamepad.back;
            case XboxControllerConstants.kLeftBumper:
                return mGamepad.left_bumper;
            case XboxControllerConstants.kRightBumper:
                return mGamepad.right_bumper;
            case XboxControllerConstants.kLeftStickButton:
                return mGamepad.left_stick_button;
            case XboxControllerConstants.kRightStickButton:
                return mGamepad.right_stick_button;
            case XboxControllerConstants.kLeftTrigger:
                //Dragons: on utilise left_trigger comme un bouton true/false
                return mGamepad.left_trigger > 0;
            case XboxControllerConstants.kRightTrigger:
                //Dragons: on utilise right_trigger comme un bouton true/false
                return mGamepad.right_trigger > 0;
            default:
                return false;
        }
    }

}
