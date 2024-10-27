package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RingLauncher")
public class RingLauncher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BLM");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRM");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BRM");

        DcMotor leftSpinner = hardwareMap.get(DcMotor.class, "left");
        DcMotor rightSpinner = hardwareMap.get(DcMotor.class, "right");
        Servo trigger = hardwareMap.get(Servo.class, "servo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (!isStopRequested()){
            double forward = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            frontLeft.setPower(forward + rotate);
            backLeft.setPower(forward + rotate);
            frontRight.setPower(forward - rotate);
            backRight.setPower(forward - rotate);

            double spinner = gamepad1.right_trigger;

            leftSpinner.setPower(spinner);
            rightSpinner.setPower(spinner);

            if(gamepad1.a){
                trigger.setPosition(1);
            }else{
                trigger.setPosition(0);
            }

        }
    }
}
