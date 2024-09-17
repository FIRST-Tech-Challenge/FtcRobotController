package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bucket {
    private String SERVO_NAME = "bucketServo";

    private Servo servo;

    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get(SERVO_NAME);
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
    }
}
