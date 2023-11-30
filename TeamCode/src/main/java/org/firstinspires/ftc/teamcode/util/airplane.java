package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.extension;

import java.security.cert.Extension;

public class airplane {
    public static boolean planeb=false;
    public static void run(extension extend, Gamepad gamepad1, Servo plane) {
        if (gamepad1.touchpad_finger_1) {
            planeb = true;
        } else if (gamepad1.touchpad_finger_2) {
            planeb = false;
        }
        if (planeb) {
            extend.setTilt(((int) Range.scale(gamepad1.touchpad_finger_1_y, -1, 1, 0, 400)));
            if (gamepad1.touchpad_finger_2) plane.setPosition(0);
        }
    }
}
