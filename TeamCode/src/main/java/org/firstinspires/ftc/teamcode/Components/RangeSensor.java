package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.angle;
import static java.lang.Math.cos;

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

    public double getDistance(AnalogInput ultrasonic) {
        double rawValue = ultrasonic.getVoltage();
        double voltage_scale_factor = (ultrasonic.getVoltage()*90) - 13.5;
        return rawValue * voltage_scale_factor * 0.0492;
    }
    public double[] getLocation(){
        double[] pos = {0,0};
        pos[1] = cos(angle)*getDistance(ultrasonicFront)-47;
        pos[0] = cos(angle)*getDistance(ultrasonicLeft);
        return pos;
    }
}