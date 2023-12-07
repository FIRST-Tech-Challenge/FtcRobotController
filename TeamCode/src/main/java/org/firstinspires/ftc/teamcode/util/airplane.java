package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.extension;



public class airplane {
    public static boolean planeb=false;
    public static boolean state=false;
    public static void run(extension extend, Gamepad gamepad1, Servo plane, boolean runmode) {
        if (gamepad1.start&& state!=gamepad1.start&&!runmode) {
            if (!planeb) {
                planeb = true;
            } else {
                planeb = false;
            }
        }
        if (gamepad1.triangle&& state!=gamepad1.start&&runmode) {
            if (!planeb) {
                planeb = true;
            } else {
                planeb = false;
            }
        }
        if (planeb) {
           // extend.setTilt(((int) Range.scale(gamepad1.touchpad_finger_1_y, -1, 1, 0, 400)));
            plane.setPosition(1);
        }
        state = gamepad1.start;
    }
}
