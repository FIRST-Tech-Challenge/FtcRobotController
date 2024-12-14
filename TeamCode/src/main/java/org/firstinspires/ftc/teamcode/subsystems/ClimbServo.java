package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClimbServo {
    private Servo leftServo, rightServo;

    public ClimbServo(HardwareMap hw, String leftServoName, String rightServoName)
    {
        leftServo = hw.get(Servo.class, leftServoName);
        rightServo = hw.get(Servo.class, rightServoName);
    }

    public void openServo()
    {
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }

    public void closeServo()
    {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }
}
