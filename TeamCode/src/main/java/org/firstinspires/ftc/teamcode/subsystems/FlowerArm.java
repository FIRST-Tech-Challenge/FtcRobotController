package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerArm {
    Servo ServoRight;
    Servo ServoLeft;

    public void init(HardwareMap hardwareMap) {
        ServoRight = hardwareMap.get(Servo.class, "ServoRight");
        ServoLeft = hardwareMap.get(Servo.class, "ServoLeft");
    }
    public void BallIn(){
        ServoRight.setPosition(0.7);
        ServoLeft.setPosition(0.3);
    }
    public void BallOut(){
        ServoRight.setPosition(0.3);
        ServoLeft.setPosition(0.7+);
    }
}
