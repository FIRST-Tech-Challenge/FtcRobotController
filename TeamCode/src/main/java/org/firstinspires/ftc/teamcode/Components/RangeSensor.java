package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;



public class RangeSensor {

    private AnalogInput ultrasonicFront;
    private AnalogInput ultrasonicRight;
    private AnalogInput ultrasonicLeft;


    public RangeSensor(LinearOpMode opMode) {
        ultrasonicFront =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
        ultrasonicLeft =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
//        ultrasonicRight =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
    }

    public double getDistance(boolean front) {
        AnalogInput ultrasonic;
        if(front){
            ultrasonic = ultrasonicFront;
        }
        else{
            ultrasonic = ultrasonicLeft;
        }
        double rawValue = ultrasonic.getVoltage();
        //0.41 = 24, 0.68 = 48
        //.27 =24
        double voltage_scale_factor = (ultrasonic.getVoltage()*90) - 13.5;
        return rawValue * 24/.27 -12.444;
    }
    public double getVoltage(boolean front) {
        AnalogInput ultrasonic;
        if(front){
            ultrasonic = ultrasonicFront;
        }
        else{
            ultrasonic = ultrasonicLeft;
        }
        return ultrasonic.getVoltage();
    }
    public double[] getLocation(){
        double[] pos = {0,0};
            pos[1] = 47-getDistance(false);
//            pos[0] = getDistance(false);
        return pos;
    }
}