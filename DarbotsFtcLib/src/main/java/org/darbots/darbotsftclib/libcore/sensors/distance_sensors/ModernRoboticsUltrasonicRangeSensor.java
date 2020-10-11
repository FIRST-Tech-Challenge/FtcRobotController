package org.darbots.darbotsftclib.libcore.sensors.distance_sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.darbots.darbotsftclib.libcore.templates.sensors.DarbotsDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ModernRoboticsUltrasonicRangeSensor extends DarbotsDistanceSensor {
    private ModernRoboticsI2cRangeSensor m_DistanceSensor;
    private double m_LastReadingCM = DarbotsDistanceSensor.DISTANCE_INVALID;

    public ModernRoboticsUltrasonicRangeSensor(ModernRoboticsI2cRangeSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public ModernRoboticsUltrasonicRangeSensor(ModernRoboticsUltrasonicRangeSensor sensor){
        this.m_DistanceSensor = sensor.m_DistanceSensor;
        this.m_LastReadingCM = sensor.m_LastReadingCM;
    }

    public ModernRoboticsI2cRangeSensor getDistanceSensor(){
        return this.m_DistanceSensor;
    }

    public void setDistanceSensor(ModernRoboticsI2cRangeSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public double getDistanceInCM(){
        return this.m_LastReadingCM;
    }

    @Override
    public void updateStatus() {
        this.m_LastReadingCM = this.m_DistanceSensor.cmUltrasonic();
        if(this.m_LastReadingCM ==  ModernRoboticsI2cRangeSensor.distanceOutOfRange){
            this.m_LastReadingCM = DISTANCE_INVALID;
        }else{
            this.m_LastReadingCM *= this.ActualDistanceFactor;
        }
    }
}
