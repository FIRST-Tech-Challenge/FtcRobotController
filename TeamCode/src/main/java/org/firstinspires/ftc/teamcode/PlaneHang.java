package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PlaneHang {
    Servo droneServo;
    boolean droneToggle = true;

    public PlaneHang(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");
    }

    // This function controls the drone.
    // The first input is the button used to control the drone.
    // The second input is the time the function uses to space out inputs.
    public void launchDrone(boolean button, ElapsedTime time) {
        if (button && time.time() > .75 && !droneToggle) {
            droneToggle = true;
            time.reset();
            droneServo.setPosition(0.0);

        } else if (button && time.time() > .75 && droneToggle) {
            droneToggle = false;
            time.reset();
            droneServo.setPosition(1.0);
        }
    }
}
