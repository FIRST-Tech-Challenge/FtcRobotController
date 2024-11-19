package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public interface Chassis {
    default void delay(double time){
        // used to stall program
        // used in many different applications and cloned to the auto classes
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < time){
            // stal program
        }
    }
    default DistanceSensor tryDeclareDistanceSensor(String distanceSensorName, HardwareMap hw){
        try {
            return hw.get(DistanceSensor.class, distanceSensorName);
        } catch (Exception e) {
            return null;
        }
    }
}