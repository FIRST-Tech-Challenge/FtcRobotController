package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {
    private DigitalChannel pin0;
    private DigitalChannel pin1;
    private String alliance;
    public ColorSensor(HardwareMap hardwareMap, String alliance){
        this.pin0 = hardwareMap.digitalChannel.get("color0");
        this.pin1 = hardwareMap.digitalChannel.get("color1");
        this.alliance = alliance;
    }

    public String getColor(){
        if(pin0.getState() && pin1.getState()){
            return "Yellow";
        }else if(pin0.getState() && !pin1.getState()){
            return "Blue";
        }else if(!pin0.getState() && pin1.getState()){
            return "Red";
        }else{
            return null;
        }
    }

}
