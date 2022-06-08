package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.angle;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;



public class RangeSensor {

    private AnalogInput ultrasonicFront;
    private AnalogInput ultrasonicRight;
    private AnalogInput ultrasonicLeft;


    public RangeSensor(LinearOpMode opMode) {
        ultrasonicFront =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
        ultrasonicLeft =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight =  opMode.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
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
            pos[1] = 53.5-getDistance(true);
            pos[0] = getDistance(false)-0.5;
            //hypot
        double hypot =0;
        if(abs(angle)<5){
            hypot = 10000;
        }else {
            hypot = pos[0] / sin(angle * PI / 180);
        }
            pos[0]*=(hypot-6)/hypot;

        return pos;
    }
}