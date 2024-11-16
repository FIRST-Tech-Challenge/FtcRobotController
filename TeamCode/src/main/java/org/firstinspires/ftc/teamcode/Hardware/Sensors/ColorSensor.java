package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;


import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor extends ColorSensorComposition {
    public com.qualcomm.robotcore.hardware.ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;
    HardwareMap hardwareMap = null;

   public ColorSensor(HardwareMap hardwareMap){
       this.hardwareMap = hardwareMap;
       this.colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
       this.distanceSensor = (DistanceSensor) hardwareMap.get(ColorSensor.class , "DistanceSensor");

       final int colorTolerance = 180;
   }
   /*  sensor 1, 2, 3
    *  1 : checks distance to see if claw should grab
    *  2 : checks to see if piece in intake is less than 2 mm
    *  3 : checks to see if second piece is in intake
    */
    public void ColorSensing(boolean red) {
        //red as in team color

        //waitForStart();

        while(linearOpMode.opModeIsActive()){
            //color sens 2.
            if(distanceSensor.getDistance(DistanceUnit.MM) <= 10){
                // 3 outtake second piece/check color
            }
            if(distanceSensor.getDistance(DistanceUnit.MM) <= 2){
                // 2 bring claw motor down
            }
            //if(colorSensor.red() > colorTolerance) {
                // 1 this is ON claw
            //}
        }
    }

}
//red()
//
//Returns the red color value detected by the sensor.
//green()
//
//Returns the green color value detected by the sensor.
//blue()
//
//Returns the blue color value detected by the sensor.
//alpha()
//
//Returns the overall light intensity, useful for detecting light and dark areas. Includes all RGB channels.
//argb()
//
//Returns a combined ARGB (alpha, red, green, blue) color value, useful for compact color information.
//getNormalizedColors()
//
//Returns normalized RGB values, useful for color detection under varying lighting conditions. Values are adjusted to be on a 0-1 scale.
//readUnsignedByte(register)
//
//Reads a single unsigned byte from a specified I2C register. Useful for accessing raw sensor data directly.
//Rev2mDistanceSensor Class (for distance/proximity data, implemented if you’re using proximity sensing capabilities)
//getDistance(DistanceUnit unit)
//
//Returns the distance to an object in the specified unit (e.g., DistanceUnit.CM or DistanceUnit.MM). This is useful for detecting proximity.
//getDeviceName()
//
//Returns the name of the sensor, often used for debugging or identifying sensors in code.
//getManufacturer()
//
//Returns the manufacturer information, useful for confirming the correct sensor in configurations.
//initialize()
//
//Initializes the distance sensor, often used at the start of an autonomous program to prepare the sensor for operation.