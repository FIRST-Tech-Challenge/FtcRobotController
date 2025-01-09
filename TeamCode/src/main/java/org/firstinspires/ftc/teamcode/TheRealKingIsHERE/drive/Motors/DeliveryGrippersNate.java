package org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeliveryGrippersNate {
    public Servo grippers;
    public HardwareMap hardwareMap;
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.hardwareMap = hwMap;
        grippers = hardwareMap.get(Servo.class, "deliveryGrippers");
    }

    public void setPosition(double position){
        grippers.setPosition(position);
    }

    public double getPosition(){
        return grippers.getPosition();
    }
}
