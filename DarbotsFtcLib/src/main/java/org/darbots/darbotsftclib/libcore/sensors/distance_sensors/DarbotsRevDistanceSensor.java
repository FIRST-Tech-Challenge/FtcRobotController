package org.darbots.darbotsftclib.libcore.sensors.distance_sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.darbots.darbotsftclib.libcore.templates.sensors.DarbotsDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DarbotsRevDistanceSensor extends DarbotsDistanceSensor {
    private DistanceSensor m_DistanceSensor;
    private double m_LastReadingCM = DarbotsDistanceSensor.DISTANCE_INVALID;

    public DarbotsRevDistanceSensor(DistanceSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public DarbotsRevDistanceSensor(DarbotsRevDistanceSensor sensor){
        this.m_DistanceSensor = sensor.m_DistanceSensor;
        this.m_LastReadingCM = sensor.m_LastReadingCM;
    }

    public DistanceSensor getDistanceSensor(){
        return this.m_DistanceSensor;
    }

    public void setDistanceSensor(DistanceSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public double getDistanceInCM(){
        return this.m_LastReadingCM;
    }

    @Override
    public void updateStatus() {
        this.m_LastReadingCM = this.m_DistanceSensor.getDistance(DistanceUnit.CM);
        if(this.m_LastReadingCM == DistanceSensor.distanceOutOfRange){
            this.m_LastReadingCM = DISTANCE_INVALID;
        }else{
            this.m_LastReadingCM *= this.ActualDistanceFactor;
        }
    }
}
