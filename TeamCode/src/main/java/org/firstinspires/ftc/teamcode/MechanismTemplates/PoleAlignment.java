package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PoleAlignment
{
    public Servo retractServo;
    public boolean isUp;
    public static double raise=.855;
    public static double lower=.41;

    public PoleAlignment(HardwareMap hardwareMap) {
        retractServo=hardwareMap.get(Servo.class, "poleAlignment"); //port 5
        isUp = false;
        retractServo.setPosition(lower);
    }


    public void raiseServo() {
        retractServo.setPosition(raise);
    }

    public void toggleAlignmentDevice() {
        if (isUp) {
            retractServo.setPosition(lower);
        }
        else {
            retractServo.setPosition(raise);
        }
        isUp = !isUp;
    }
}

