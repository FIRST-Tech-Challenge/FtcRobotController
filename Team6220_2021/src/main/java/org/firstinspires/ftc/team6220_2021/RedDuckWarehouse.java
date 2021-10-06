package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Red Duck/Warehouse", group = "Autonomous")
public class RedDuckWarehouse extends LinearOpMode {

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

        motorBackLeft.setPower(0.8);
        motorBackRight.setPower(0.8);
        motorFrontLeft.setPower(0.8);
        motorFrontRight.setPower(0.8);

        pauseMillis(500);

        motorDuck.setPower(x);
        pauseMillis(150);
        x += 0.05;
        telemetry.addData("duckPower", motorDuck.getPower());
        telemetry.update();
        if (x >= 0.95) {
            pauseMillis(600);
            motorDuck.setPower(-.1);
            pauseMillis(30);
            motorDuck.setPower(0);
            x = 0.7;

            motorBackLeft.setPower(0.8);
            motorFrontRight.setPower(0.8);

            pauseMillis(200);

            motorBackLeft.setPower(-0.8);
            motorBackRight.setPower(-0.8);
            motorFrontLeft.setPower(-0.8);
            motorFrontRight.setPower(-0.8);

            pauseMillis(2000);

        }
    }
        public void pauseMillis ( double time){
            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
                idle();
            }
        }
    }