package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MiscMechs {
    Servo droneServo;
    ;
    boolean droneToggle = false;

    public MiscMechs(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");
    }

    
    public void launchDrone(boolean button, ElapsedTime time) {
        if (button && time.time() > .75 && !droneToggle) {
            droneToggle = true;
            time.reset();


        } else if (button && time.time() > .75 && droneToggle) {
            droneToggle = false;
            time.reset();

        }
    }
}
