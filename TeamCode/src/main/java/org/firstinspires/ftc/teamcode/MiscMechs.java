package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MiscMechs {
    Servo droneServo;
    boolean droneToggle = false;

    public MiscMechs(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");
    }

    // This function controls the drone.
    // The first input is the button used to control the drone.
    // The second input is the time the function uses to space out inputs.
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
