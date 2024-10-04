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
    private boolean servoTriggered;

    public Bucket(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get(SERVO_NAME);
        servoTriggered = false;
    }

    public void update(Gamepad gamepad){
        if(gamepad.y && servoTriggered){
            servo.setPosition(1);
            servoTriggered = !servoTriggered;
        } else if(gamepad.y && !servoTriggered){
            servo.setPosition(0);
            servoTriggered = !servoTriggered;
        }
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
    }
}
