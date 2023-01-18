package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdoPod
{
    public static Servo retractServo;
    public static double down=.85;
    public static double retract=.65;
    public OdoPod(HardwareMap hardwareMap) {
        retractServo=hardwareMap.get(Servo.class, "odoServo");//port 3
        retractServo.setPosition(down);
    }
    public static void retract()
    {
        retractServo.setPosition(retract);
    }
}

