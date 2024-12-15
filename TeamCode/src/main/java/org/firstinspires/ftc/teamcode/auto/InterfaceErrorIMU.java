package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.IMU;

public class InterfaceErrorIMU {

    private final String name;
    private IMU imu;

    public InterfaceErrorIMU(String name){
        this.name = name;
    }


    public String getName() {
        return name;
    }
    public void setImu(IMU imu){
        this.imu = imu;
    }
    public IMU getImu() {
        return imu;
    }
}
