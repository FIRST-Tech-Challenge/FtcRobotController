package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawLeft{
    public Servo servo_left;
    HardwareMap hardwareMap;


    public void init(HardwareMap ahwMap) {
        servo_left = hardwareMap.get(Servo.class, "left_servo");
        hardwareMap = ahwMap;

    }

}
