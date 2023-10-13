package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeHingeCRServo {

    static CRServo hingeCRServo;

    public static void intakeHingeServoInit(CRServo hingey) {
        hingeCRServo = hingey;
    }

    public static void manualHinge(boolean a, boolean b) {
        if(a) {
            hingeCRServo.setPower(0.2);
        } else if(b) {
            hingeCRServo.setPower(-0.2);
        } else {
            hingeCRServo.setPower(0);
        }
    }
}
