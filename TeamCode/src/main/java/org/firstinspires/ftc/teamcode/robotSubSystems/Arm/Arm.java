package org.firstinspires.ftc.teamcode.robotSubSystems.Arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public static Servo armServo;
    public static void init(HardwareMap hardwareMap) {
        armServo = hardwareMap.servo.get("armServo");

    }
    public static void operate(ArmState state) {
        switch (state) {
            case INTAKE:
                armServo.setPosition(0);
                break;
            case DEPLETE:
                armServo.setPosition(0.5);
                break;
        }
    }
}