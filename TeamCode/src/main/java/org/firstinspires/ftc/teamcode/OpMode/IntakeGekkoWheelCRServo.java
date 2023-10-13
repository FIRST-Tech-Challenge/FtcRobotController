package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeGekkoWheelCRServo {

    static CRServo axleCRServo;

    public static void gekkoWheelInit(CRServo spinny) {
        axleCRServo = spinny;
    }

    public static void runWheels(boolean a) {

        if(a) {
            axleCRServo.setPower(1);
        } else {
            axleCRServo.setPower(0);
        }
    }
}
