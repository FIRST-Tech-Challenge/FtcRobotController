package org.firstinspires.ftc.teamcode.Hardware.Sensors;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Color extends ColorSensorComposition {
    public com.qualcomm.robotcore.hardware.ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;
    HardwareMap hardwareMap = null;

   public Color(HardwareMap hardwareMap){
       this.hardwareMap = hardwareMap;
       this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
       //this.distanceSensor = hardwareMap.get(DistanceSensor.class , "DistanceSensor");

       final int colorTolerance = 180;
   }
   public boolean isRed(){
       return colorSensor.red() > 0.5;
   }
   public boolean isBlue(){
       return colorSensor.blue() > 0.5;
   }
}
