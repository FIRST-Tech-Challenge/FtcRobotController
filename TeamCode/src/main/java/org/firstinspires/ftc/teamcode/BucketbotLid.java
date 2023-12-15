package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketbotLid {
    Servo servo;

    public BucketbotLid(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "lidServo");
    }

    public void openLid(boolean open) {
        if (open) {
            servo.setPosition(1.0);
        }
        else if(!open) {
            servo.setPosition(0.0);
        }
    }

}
