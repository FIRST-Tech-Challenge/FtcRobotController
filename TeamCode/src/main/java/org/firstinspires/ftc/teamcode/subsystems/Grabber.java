package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber extends SubsystemBase {

    private CRServo servo;
    private Telemetry tm;

    public Grabber(HardwareMap hardwareMap, Telemetry telemetry){

        tm = telemetry;
        //TODO: fix this name from config
        servo = hardwareMap.get(CRServo.class, "grabber");
        tm.addData("happy", "happy");
    }

    public void pickup() {
        servo.setPower(1.0);
    }

    public void drop() {
        servo.setPower(-1.0);
    }

    public void stop() {
        servo.setPower(0.0);
    }

}