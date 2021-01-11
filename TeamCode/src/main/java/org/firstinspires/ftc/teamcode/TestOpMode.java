package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestOpMode extends LinearOpMode{
    /*private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;*/
    private DcMotor rampLeft;
    private DcMotor rampRight;

    public void runOpMode(){
        /*frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");*/

        rampLeft = hardwareMap.get(DcMotor.class, "rampLeft");
        rampRight = hardwareMap.get(DcMotor.class, "rampRight");

        waitForStart();

        double throttle = 0;
        double turnL = 0;
        double turnR = 0;
        double throttle2 = 0;

        /*backRight.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);*/

        rampLeft.setPower(0);
        rampRight.setPower(0);

        while (opModeIsActive()){
            throttle = this.gamepad1.left_stick_y;
            turnL = this.gamepad1.right_stick_x;
            turnR = -this.gamepad1.right_stick_x;
            throttle = throttle*.25;



            rampLeft.setPower(throttle);
            rampRight.setPower(throttle);

            /*frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            frontLeft.setPower(turnL);
            frontRight.setPower(turnR);
            backLeft.setPower(turnL);
            backRight.setPower(turnR);*/
        }
    }
}
