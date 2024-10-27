package org.firstinspires.ftc.teamcode.Alan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp
public class TankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");
        Servo swarsh = hardwareMap.servo.get("s");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double INCHES_PER_TICK = 40 / -13510.0 * (40.0 / 40.3612);


        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double Left = gamepad1.left_stick_y;
            double Right = gamepad1.right_stick_y;
            leftFront.setPower(-Left);
            rightFront.setPower(-Right);
            leftBack.setPower(-Left);
            rightBack.setPower(-Right);
            if (gamepad1.a) {
                swarsh.setPosition(1.0);
            } else {
                swarsh.setPosition(0.0);
            }

//            double prevTime = System.currentTimeMillis();
//            double prevDistance = 0;
//            while (true) {
//                double currentTime = System.currentTimeMillis();
//                double currentDistance = rightBack.getCurrentPosition() * INCHES_PER_TICK;
//
//                double error = 20 - currentDistance;
//                double power = (Math.min(0.05, error * 0.05));
//
//                double speed; // =
//
//
//                leftFront.setPower(power);
//                rightFront.setPower(power);
//                leftBack.setPower(power);
//                rightBack.setPower(power);
//
//                if (Math.abs(error) < 0.05) {
//                    break;
//                }
//                prevTime = currentTime;
//            }
            telemetry.addData("done?", "done");
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);


            telemetry.addData("Motor: ", gamepad1.left_stick_y);
            telemetry.addData("encoder", rightBack.getCurrentPosition() * INCHES_PER_TICK);
            telemetry.update();
        }
    }
}

