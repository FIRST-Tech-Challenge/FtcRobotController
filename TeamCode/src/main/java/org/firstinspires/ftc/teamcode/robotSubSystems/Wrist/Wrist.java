package org.firstinspires.ftc.teamcode.robotSubSystems.Wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public static Servo rightWristServo;
    public static Servo leftWristServo;


    public static void init(HardwareMap hardwareMap) {
        rightWristServo = hardwareMap.servo.get("rightWristServo");
        leftWristServo = hardwareMap.servo.get("leftWristServo");
    }

    public static void operate(WristState state) {
        switch (state) {
            case TRANSFER:
                leftWristServo.setPosition(0.02);
                rightWristServo.setPosition(1);
                break;
            case INTAKE:
                leftWristServo.setPosition(0.75);
                rightWristServo.setPosition(0.24);
                break;
            case DEPLETE:
                leftWristServo.setPosition(0.67);
                rightWristServo.setPosition(0.35);
        }
    }
}
