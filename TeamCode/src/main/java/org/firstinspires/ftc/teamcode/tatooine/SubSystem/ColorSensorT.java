package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.tatooine.utils.SampleColor;

public class ColorSensorT {
    SampleColor sampleColor;
    NormalizedColorSensor colorSensor = null;

    public ColorSensorT(HardwareMap hardwareMap, SampleColor sampleColor){
        this.sampleColor = sampleColor;
        colorSensor =  hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
    }


//    public boolean isRightColor(){
//
//    }

}
