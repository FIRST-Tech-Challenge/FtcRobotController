package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SlideTest", group = "TeleOp")

public class Krishiv_Test extends OpMode {



    DcMotor Slide;





    @Override
    public void init() {

        Slide = hardwareMap.dcMotor.get("Slide");


    }


    @Override
    public void loop() {



        if (gamepad2.left_bumper) { //down
            Slide.setPower(-0.4); //(-0.2)
        } else if (gamepad2.right_bumper) { //up
            Slide.setPower(0.9);
        } else {
            Slide.setPower(0.3);
        }

    }
}