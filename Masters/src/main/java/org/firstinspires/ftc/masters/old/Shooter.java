package org.firstinspires.ftc.masters.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.Component;

@Disabled
public class Shooter implements Component {
    private HardwareMap hardwareMap = null;
    Servo planeRaise;
    ElapsedTime time=null;

    public Shooter(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        initializeHardware();
    }
    public void initializeHardware(){

        planeRaise = hardwareMap.servo.get("planeRaise");
        planeRaise.setPosition(CSCons.droneFlat);
    }

    public void update(Gamepad gamepad){
        if (gamepad.touchpad_finger_2) {
            shoot();
            time= new ElapsedTime();
        }

        if (time!=null && time.milliseconds()>500){
            planeRaise.setPosition(CSCons.droneFlat);
            time=null;
        }
    }

    public void initializeDrone(){
        planeRaise.setPosition(CSCons.droneFlat);
    }


    public void shoot(){
        planeRaise.setPosition(CSCons.droneShooting);
    }
}
