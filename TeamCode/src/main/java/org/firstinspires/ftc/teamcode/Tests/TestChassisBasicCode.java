package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestChassisBasicCode extends OpMode {

    private DcMotor Bleft;
    private DcMotor Bright;
    private DcMotor Fleft;
    private DcMotor Fright;




    public void init(){
        Bleft = hardwareMap.dcMotor.get("Bleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Fleft  = hardwareMap.dcMotor.get("Fleft");
        Fright = hardwareMap.dcMotor.get("Fright");




    }
    public void loop() {

        if (gamepad1.right_stick_y != 0.0) {
            Bleft.setPower(gamepad1.right_stick_y);
            Bright.setPower(gamepad1.right_stick_y);
            Fright.setPower(gamepad1.right_stick_y);
            Fleft.setPower(gamepad1.right_stick_y);


        }
        else if(gamepad1.left_stick_x !=0.0) {
            Bleft.setPower(-1*gamepad1.left_stick_x);
            Bright.setPower(gamepad1.left_stick_x);
            Fright.setPower(-1*gamepad1.left_stick_x);
            Fleft.setPower(gamepad1.left_stick_x);

        }
        else{
            Bleft.setPower(0);
            Bright.setPower(0);
            Fright.setPower(0);
            Fleft.setPower(0);
        }

        }




    }


