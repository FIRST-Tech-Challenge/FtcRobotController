package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue Duck/Warehouse", group = "Autonomous")
public class BlueDuckWarehouse extends LinearOpMode {

    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    double x = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        motorBackLeft.setPower(-0.6);
        motorBackRight.setPower(-0.6);
        motorFrontLeft.setPower(-0.6);
        motorFrontRight.setPower(-0.6);

        pauseMillis(1000);

        motorBackRight.setPower(0.6);
        motorFrontRight.setPower(0.6);
        motorBackLeft.setPower(-0.6);
        motorFrontLeft.setPower(-0.6);

        pauseMillis(700);

        motorBackLeft.setPower(-0.6);
        motorBackRight.setPower(-0.6);
        motorFrontLeft.setPower(-0.6);
        motorFrontRight.setPower(-0.6);
        pauseMillis(1000);

        while (true) {
            motorDuck.setPower(x);
            pauseMillis(150);
            x += 0.05;
            telemetry.addData("duckPower", motorDuck.getPower());
            telemetry.update();
            if (x>=0.85){
                pauseMillis(1000);
                motorDuck.setPower(-.1);
                pauseMillis(30);
                motorDuck.setPower(0);
                x=0.7;
                break;
            }
        }

        motorBackLeft.setPower(0.6);
        motorBackRight.setPower(0.6);
        motorFrontLeft.setPower(0.6);
        motorFrontRight.setPower(0.6);

        pauseMillis(500);

        motorBackRight.setPower(0.6);
        motorFrontRight.setPower(0.6);
        motorBackLeft.setPower(-0.6);
        motorFrontLeft.setPower(-0.6);

            pauseMillis(500);

            motorBackLeft.setPower(-0.8);
            motorBackRight.setPower(-0.8);
            motorFrontLeft.setPower(-0.8);
            motorFrontRight.setPower(-0.8);

            pauseMillis(4000);

        }


        public void pauseMillis ( double time){
            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
                idle();
            }
        }
    }