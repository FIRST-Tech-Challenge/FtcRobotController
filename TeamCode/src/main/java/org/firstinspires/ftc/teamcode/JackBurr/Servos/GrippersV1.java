package org.firstinspires.ftc.teamcode.JackBurr.Servos;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GrippersV1 {
    public Servo grippers;
    public HardwareMap hardwareMap;
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.hardwareMap = hwMap;
        grippers = hardwareMap.get(Servo.class, "grippers");
    }

    public void setPosition(double position){
        grippers.setPosition(position);
    }
}
