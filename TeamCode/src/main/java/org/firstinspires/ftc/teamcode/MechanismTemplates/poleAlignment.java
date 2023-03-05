package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class poleAlignment
{
    public static Servo retractServo;
    public static double raise=.55;
    public static double lower=.13;
    public poleAlignment(HardwareMap hardwareMap) {
        retractServo=hardwareMap.get(Servo.class, "poleAlignment"); //port 5
    }
    public static void lower()
    {
        retractServo.setPosition(lower);
    }

    public static void raiseServo()
    {
        retractServo.setPosition(raise);
    }

    public static void toggleAlignmentDevice() {
        if (retractServo.getPosition() == raise) {
            retractServo.setPosition(lower);
        }
        else {
            retractServo.setPosition(raise);
        }
    }
}

