package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoManage {
    private Servo servo;
    private Telemetry telemetry;
    public ServoManage(HardwareMap hardwareMap, Telemetry telemetryGet, String name) {
        servo = hardwareMap.get(Servo.class, name);

        telemetry = telemetryGet;
    }
    public void servoPosition1() {
        servo.setPosition(0);
    }
    public void servoPosition2() {
        servo.setPosition(1);
    }
    public void servoPositionX(double pos) {
        servo.setPosition(pos);
    }
    public double getPos() {
        return servo.getPosition();
    }
}