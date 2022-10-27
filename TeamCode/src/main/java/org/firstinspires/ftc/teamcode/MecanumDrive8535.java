package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumDrive8535 extends LinearOpMode {
    private DcMotor bottomLeftMotor = null;
    private DcMotor topLeftMotor = null;
    private DcMotor bottomRightMotor = null;
    private DcMotor topRightMotor = null;
    public Servo Claw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Init");
        telemetry.update();

        bottomLeftMotor = hardwareMap.get(DcMotor.class, "bottomLeftMotor");
        topLeftMotor = hardwareMap.get(DcMotor.class, "topLeftMotor");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "bottomRightMotor");
        topRightMotor = hardwareMap.get(DcMotor.class, "topRightMotor");
        Claw = hardwareMap.get(Servo.class, "Claw");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a == true) {
                bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                topLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                bottomRightMotor.setDirection(DcMotor.Direction.REVERSE);
                topRightMotor.setDirection(DcMotor.Direction.REVERSE);
                bottomLeftMotor.setPower(0.5);
                topLeftMotor.setPower(0.5);
                bottomRightMotor.setPower(0.5);
                topRightMotor.setPower(0.5);
            } else if (gamepad1.y == true) {
                bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                bottomRightMotor.setDirection(DcMotor.Direction.FORWARD);
                topRightMotor.setDirection(DcMotor.Direction.FORWARD);
                bottomLeftMotor.setPower(0.5);
                topLeftMotor.setPower(0.5);
                bottomRightMotor.setPower(0.5);
                topRightMotor.setPower(0.5);
            } else if (gamepad1.b == true) {
                bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                topLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                bottomRightMotor.setDirection(DcMotor.Direction.FORWARD);
                topRightMotor.setDirection(DcMotor.Direction.REVERSE);
                bottomLeftMotor.setPower(0.5);
                topLeftMotor.setPower(0.5);
                bottomRightMotor.setPower(0.5);
                topRightMotor.setPower(0.5);
            } else if (gamepad1.x == true) {
                bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                bottomRightMotor.setDirection(DcMotor.Direction.REVERSE);
                topRightMotor.setDirection(DcMotor.Direction.FORWARD);
                bottomLeftMotor.setPower(0.5);
                topLeftMotor.setPower(0.5);
                bottomRightMotor.setPower(0.5);
                topRightMotor.setPower(0.5);
            } else {
                bottomLeftMotor.setPower(0.0);
                topLeftMotor.setPower(0.0);
                bottomRightMotor.setPower(0.0);
                topRightMotor.setPower(0.0);
            }
            if (gamepad1.b) {
                Claw.setPosition(0.30);

            } else if (gamepad1.a) {
                Claw.setPosition(0.80);

                telemetry.addData("Up", gamepad1.dpad_up);
                telemetry.addData("Down", gamepad1.dpad_down);
                telemetry.addData("Left", gamepad1.dpad_left);
                telemetry.addData("Right", gamepad1.dpad_right);
                telemetry.addData("Close", gamepad1.a);
                telemetry.addData("Open", gamepad1.b);
                telemetry.update();
            }

        }
    }
}