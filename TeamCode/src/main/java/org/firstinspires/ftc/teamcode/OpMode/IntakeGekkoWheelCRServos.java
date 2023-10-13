package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeGekkoWheelCRServos {

    static CRServo axleCRServo;

    public static void gekkoWheelInit(CRServo spinny) {
        axleCRServo = spinny;
    }

    public static void runWheels(boolean dpadUp) {
        axleCRServo.setPower(1.0);
    }
}
