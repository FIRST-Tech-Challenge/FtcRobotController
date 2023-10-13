package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeHingeServos {

    static CRServo hingeCRServo;

    public static void intakeHingeServoInit(CRServo hingey) {
        hingeCRServo = hingey;
    }

    public static void manualHinge(boolean a, boolean b) {
        if(a) {
            hingeCRServo.setPower(0.1);
        } else if(b) {
            hingeCRServo.setPower(-0.1);
        } else {
            hingeCRServo.setPower(0);
        }
    }
}
