package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.CSCons;

public class Shooter implements Component{
    private HardwareMap hardwareMap = null;
    Servo planeRaise;
    public Shooter(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initializeHardware(){

        planeRaise = hardwareMap.servo.get("planeRaise");

        planeRaise.setPosition(CSCons.droneFlat);
    }

    public void update(Gamepad gamepad){
        if (gamepad.touchpad_finger_2) {
            shoot();
        }
    }

    public void shoot(){
        planeRaise.setPosition(CSCons.droneShooting);
    }
}
