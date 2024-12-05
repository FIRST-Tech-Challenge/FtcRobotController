package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo servoWrist;

    public void init(HardwareMap hardwareMap) {

        servoWrist = hardwareMap.get(Servo.class,"servoWristPosSet");

    }

    public void setServoWristPos(double wristInput) {

        servoWrist.setPosition(wristInput); // open

    }

}
