package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class Motor extends OpMode{
    DcMotor motor;

    public void init(){
        motor = hardwareMap.get(DcMotor.class, "FrontLeft");
    }
    public void loop(){
        if (gamepad1.left_bumper) {
            motor.setPower(1);
        }

        else {
            motor.setPower(0);
        }
    }
}