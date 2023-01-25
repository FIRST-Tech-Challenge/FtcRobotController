package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;

public class limitSwitch {
    robotConfig r;

    private static TouchSensor limitSwitch;

    public limitSwitch(robotConfig r){
        this.r = r;

        limitSwitch = r.hardwareMap.get(TouchSensor.class, "limit");
    }

    public boolean isPressed(){
        return limitSwitch.isPressed();
    }
}
