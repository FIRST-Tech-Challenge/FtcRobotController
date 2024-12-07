package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Pincher {
    private Servo axis1 = null;
    private Servo axis2 = null;
    private Servo pinch = null;
    public boolean isOpen = false;
    public void Init(HardwareMap hardwareMap) {
        axis1 = hardwareMap.get(Servo.class, "axis1");
        axis2 = hardwareMap.get(Servo.class, "axis2");
        pinch = hardwareMap.get(Servo.class, "pinch");
    }
    public void GoToReadyPosition() {
        // make arm ready to pickup sample/specimen
        axis1.setPosition(0.85);
        axis2.setPosition(1.0);

    }
    public void GoToPickupPosition() {
        // make arm ready to give sample/specimen to main claw
        axis1.setPosition(1);
        axis2.setPosition(1);
    }
    public void GoToDrivePosition() {
        // make arm ready to pickup sample/specimen
        axis1.setPosition(0.0);
        axis2.setPosition(0.0);
    }
    public void Pickup() {
        // bring arm wrist up so we can drive without issues
        axis1.setPosition(1);
        axis2.setPosition(1);
    }
    public void DropOff() {
        axis1.setPosition(0.3);
        axis2.setPosition(0.3);
    }
    public void PincherClose() {
        pinch.setPosition(0.0);
        isOpen = false;
    }
    public void PincherOpen() {
        pinch.setPosition(1);
        isOpen = true;
    }
}
