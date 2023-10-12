package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeHingeServos {

    static Servo leftHinge;
    static Servo rightHinge;

    public static void intakeHingeServoInit(Servo left, Servo right) {
        leftHinge = left;
        rightHinge = right;
    }

    public static void manualHinge() {

    }
}
