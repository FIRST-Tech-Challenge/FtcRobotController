package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.*;

public class Claw {
    private Servo servo1 = null;
    public Claw (HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "servo1");
    }

    public void open() {
       servo1.setPosition(0); //temp position
    }

    public void close() {
        servo1.setPosition(1); //temp position
    }
}
