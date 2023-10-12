package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeHingeServos {

    static CRServo leftHinge;
    static CRServo rightHinge;

    public static void intakeHingeServoInit(CRServo left, CRServo right) {
        leftHinge = left;
        rightHinge = right;
    }

    public static void manualHinge() {

    }
}
