package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ClawOpen extends OpMode{
    private Servo servo1;
    private Servo servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.servo.get("ServoClaw1");
        servo2 = hardwareMap.servo.get("ServoClaw2");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            servo2.setPosition(0.5);
            servo1.setPosition(1);
        }

        if(gamepad1.b){
            servo2.setPosition(0.5);
            servo1.setPosition(0);
        }
    }
}