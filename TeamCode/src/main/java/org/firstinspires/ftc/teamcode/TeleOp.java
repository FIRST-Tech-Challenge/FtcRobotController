package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        Servo servoTest = hardwareMap.get(Servo.class, "launch_servo");


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            double modifier = 0.5;
            rightFront.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y + gamepad1.right_stick_x) * modifier);
            leftFront.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) * modifier);
            leftBack.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) * modifier);
            rightBack.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y + gamepad1.right_stick_x) * modifier);


            // check to see if we need to move the servo.
            if (gamepad1.y) {
                // move to 0 degrees.
//                servoTest.setPosition(1);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
//            servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
//            servoTest.setPosition(1);
            }
//        telemetry.addData("Servo Position", servoTest.getPosition());
//        telemetry.addData("Target Power", tgtPower);
//        telemetry.addData("Status", "Running");
//        telemetry.update();
        }

    }
}