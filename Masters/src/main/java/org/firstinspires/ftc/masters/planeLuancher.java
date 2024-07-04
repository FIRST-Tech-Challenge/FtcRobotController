package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class planeLuancher {
    private Servo planeRaise;

    private ElapsedTime elapsedTime = null;

    public planeLuancher(HardwareMap hardwareMap){
        planeRaise = hardwareMap.servo.get("planeRaise");
    }

    public void init(){
        planeRaise.setPosition(CSCons.droneFlat);
    }

    public void rise(){
        planeRaise.setPosition(CSCons.droneShooting);
        elapsedTime = new ElapsedTime();
    }

    public void update(){
        if (elapsedTime != null){
            if (elapsedTime.time(TimeUnit.MILLISECONDS) > 500){
                planeRaise.setPosition(CSCons.droneFlat);
                elapsedTime = null;
            }
        }
    }
}
