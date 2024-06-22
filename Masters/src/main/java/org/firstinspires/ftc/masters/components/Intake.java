package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Component{
    private HardwareMap hardwareMap = null;
    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initializeHardware(){
    }
}
