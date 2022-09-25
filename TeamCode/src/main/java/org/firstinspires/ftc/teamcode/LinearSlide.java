package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LinearSlide extends OpMode{
    private DcMotor linearSlide;
    private Servo clawServo;

    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class,"LinearSlide");
        clawServo = hardwareMap.get(Servo.class,"ServoClaw1");
    }

    @Override
    public void loop() {
        linearSlide.setPower(gamepad2.left_stick_x);

        if(gamepad2.a){
            clawServo.setPosition(0.5);
        }

        if(gamepad2.b){
            clawServo.setPosition(0.5);
        }
    }
}
