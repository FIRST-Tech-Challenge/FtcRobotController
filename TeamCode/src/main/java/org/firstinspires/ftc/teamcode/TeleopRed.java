package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleopRed")

public class TeleopRed extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft, backLeft, frontRight,backRight;
        CRServo carousel;

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        carousel = hardwareMap.get(CRServo.class, "carousel");

        waitForStart();

        //turn with right stick
        if (gamepad1.right_stick_x>0.1){
            frontLeft.setPower(gamepad1.right_stick_x*MAX_SPEED);
            frontRight.setPower(gamepad1.right_stick_x*MAX_SPEED*-1);
            backLeft.setPower(gamepad1.right_stick_x*MAX_SPEED);
            backRight.setPower(gamepad1.right_stick_x*MAX_SPEED*-1);
        } else if (gamepad1.right_stick_x<0.1){
            frontLeft.setPower(gamepad1.right_stick_x*MAX_SPEED*-1);
            frontRight.setPower(gamepad1.right_stick_x*MAX_SPEED);
            backLeft.setPower(gamepad1.right_stick_x*MAX_SPEED*-1);
            backRight.setPower(gamepad1.right_stick_x*MAX_SPEED);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }

        turnDuck(carousel);


    }

    protected void turnDuck(CRServo carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(0.9);
        } else {
            carousel.setPower(0);
        }
    }
}
