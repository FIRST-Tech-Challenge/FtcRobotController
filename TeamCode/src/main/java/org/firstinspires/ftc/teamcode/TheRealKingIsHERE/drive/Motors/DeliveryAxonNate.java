package org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryAxonNate {
    public Servo axon;
    public HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        axon = hardwareMap.get(Servo.class, "deliveryServo");
    }

    public void setPosition(double position){
        axon.setPosition(position);
    }
}
