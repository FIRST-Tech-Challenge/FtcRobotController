package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MiscMechs {
    Servo droneServo;

    public MiscMechs(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");
    }

    public void launchDrone(boolean open) {
        if (open){
            droneServo.setPosition(0.1);
        }
        else {
            droneServo.setPosition(0.2);
        }
    }
}
