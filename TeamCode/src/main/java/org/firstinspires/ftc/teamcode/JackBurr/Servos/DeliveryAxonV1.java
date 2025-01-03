package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryAxonV1 {
    public Servo axon;
    public HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        axon = hardwareMap.get(Servo.class, "deliveryServo");
        axon.setPosition(0);
    }

    public void setPosition(double position){
        axon.setPosition(position);
    }
}
