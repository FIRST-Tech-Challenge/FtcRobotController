package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outake implements Component{
    private HardwareMap hardwareMap = null;
    public Outake(HardwareMap hardwareMap){
        this.hardwareMap=hardwareMap;
    }
    public void initializeHardware(){
    }
}
