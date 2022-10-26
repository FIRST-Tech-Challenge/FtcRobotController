package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class LinearSlide extends OpMode{
    private DcMotor Test1;
    private DcMotor Test2;
    //private Servo clawServo;

    @Override
    public void init() {
        Test1 = hardwareMap.get(DcMotor.class,"Test1");
        Test2 = hardwareMap.get(DcMotor.class,"Test2");
        //clawServo = hardwareMap.get(Servo.class,"ServoClaw1");
    }

    @Override
    public void loop() {
        Test1.setPower(-gamepad2.left_stick_y);
        Test2.setPower(gamepad2.left_stick_y);


      /*  if(gamepad2.a){
            clawServo.setPosition(0.5);
        }

        if(gamepad2.b){
            clawServo.setPosition(0.5);
        }*/
    }
}
