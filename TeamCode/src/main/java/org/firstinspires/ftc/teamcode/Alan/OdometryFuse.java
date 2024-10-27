package org.firstinspires.ftc.teamcode.Alan;

//test phase

import android.annotation.SuppressLint;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OdometryFuse extends LinearOpMode {
    SparkFunOTOS myOtos;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        //configureOtos();
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double INCHES_PER_TICK = 40 / -13510.0 * (40.0 / 40.3612);

        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            leftFront.setPower(forward + turn + strafe);
            rightFront.setPower(forward - turn - strafe);
            leftBack.setPower(forward + turn - strafe);
            rightBack.setPower(forward - turn + strafe);

            telemetry.addData("Encoder Pos:", rightBack.getCurrentPosition() * INCHES_PER_TICK);
            telemetry.update();
        }
    }
}
