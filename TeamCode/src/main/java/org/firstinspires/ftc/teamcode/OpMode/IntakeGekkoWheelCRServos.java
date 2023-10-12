package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeGekkoWheelCRServos {

    static CRServo leftWheel;
    static CRServo rightWheel;

    public static void gekkoWheelInit(CRServo leftServo, CRServo rightServo) {
        leftWheel = leftServo;
        rightWheel = rightServo;
    }

    public static void runWheels(boolean dpadUp) {
        leftWheel.setPower(1.0);
        rightWheel.setPower(1.0);
    }
}
