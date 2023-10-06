package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NewDriveMethod (Java)")

public class NewDriveMethod extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor attachment_1;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Speed_percentage;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");



        // Put initialization blocks here.
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Speed_percentage = 0.6;
        // Speed_percentage = 60% speed
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                double backRightPower = gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x;
                double backLeftPower = gamepad1.left_stick_y + gamepad1.left_stick_x + -gamepad1.right_stick_x;
                double frontRightPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                double frontLeftPower = gamepad1.left_stick_y + -gamepad1.left_stick_x + -gamepad1.right_stick_x;

                telemetry.addData("Power of backLeft", backLeftPower);
                telemetry.addData("Power of backRight", backRightPower);
                telemetry.addData("Power of frontLeft", frontLeftPower);
                telemetry.addData("Power of frontRight", frontRightPower);

                // highestPower is the highest value out of all of the absolute values of
                // backRightPower, backLeftPower, frontRightPower, and frontLeftPower.
                double highestPower = Math.max(Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)),
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)));

                if (highestPower > 1) {
                    backLeftPower = backLeftPower / highestPower;
                    backRightPower = backRightPower / highestPower;
                    frontLeftPower = frontLeftPower / highestPower;
                    frontRightPower = frontRightPower / highestPower;
                }

                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }
                // directional driving based on robot position
//                backRight.setPower((gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * Speed_percentage);
//                backLeft.setPower((gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * Speed_percentage);
//                frontRight.setPower((gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * Speed_percentage);
//                frontLeft.setPower((gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * Speed_percentage);
                backRight.setPower((backRightPower));
                backLeft.setPower((backLeftPower));
                frontRight.setPower((frontRightPower));
                frontLeft.setPower((frontLeftPower));
                telemetry.update();
            }
        }
    }
}