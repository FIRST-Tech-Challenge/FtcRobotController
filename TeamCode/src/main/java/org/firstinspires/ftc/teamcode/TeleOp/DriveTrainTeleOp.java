package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveTrainTeleOp extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        //Defining motors/variables
        //type   name  = (restate type) library.get("name");
        DcMotor leftFront =(DcMotor) hardwareMap.get("leftFront");
        DcMotor rightFront =(DcMotor) hardwareMap.get("rightFront");
        DcMotor leftRear =(DcMotor) hardwareMap.get("leftRear");
        DcMotor rightRear =(DcMotor) hardwareMap.get("rightRear");

        //setting direction, left or right have to be the same. Ex:
        //leftFront.setDirection 

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //brakes when no power is being given
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            //y stick value is reversed

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double multiplier;
            if (gamepad1.right_bumper) {
                multiplier = 0.6;
            }
            else multiplier = 1;
            leftFront.setPower(frontLeftPower *multiplier);
            leftRear.setPower(backLeftPower *multiplier);
            rightFront.setPower(frontRightPower *multiplier);
            rightRear.setPower(backRightPower *multiplier);
        }
    }
}

