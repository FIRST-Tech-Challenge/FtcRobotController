package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeGekkoWheelServos {

    static Servo leftWheel;
    static Servo rightWheel;

    public static void gekkoWheelInit(Servo leftServo, Servo rightServo) {
        leftWheel = leftServo;
        rightWheel = rightServo;
    }
}
