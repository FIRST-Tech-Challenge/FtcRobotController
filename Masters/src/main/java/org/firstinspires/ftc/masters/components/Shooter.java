package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter implements Component{
    private HardwareMap hardwareMap = null;
    public Shooter(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initializeHardware(){
    }
}
