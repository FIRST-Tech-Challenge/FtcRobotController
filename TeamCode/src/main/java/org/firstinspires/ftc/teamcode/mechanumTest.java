package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MechanumTest (Blocks to Java)")
public class MechanumTest extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor screwLeft;
    private DcMotor screwRight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        screwLeft = hardwareMap.get(DcMotor.class, "screwLeft");
        screwRight = hardwareMap.get(DcMotor.class, "screwRight");

        // Put initialization blocks here.
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        screwLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screwRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screwLeft.setDirection(DcMotor.Direction.FORWARD);
        screwRight.setDirection(DcMotor.Direction.FORWARD);
        screwLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screwRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                mecanum();
                Lift();
                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
                telemetry.addData("Right Stick X", gamepad1.right_stick_x);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Lift() {
        if (gamepad1.dpad_up) {
            screwLeft.setPower(1);
            screwRight.setPower(1);
        } else if (gamepad1.dpad_down) {
            screwLeft.setPower(-1);
            screwRight.setPower(-1);
        } else {
            screwLeft.setPower(0);
            screwRight.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void mecanum() {
        frontRight.setPower(Math.pow(gamepad1.left_stick_y, 3) + Math.pow(gamepad1.right_stick_x, 3) + Math.pow(gamepad1.left_stick_x, 3));
        backRight.setPower((Math.pow(gamepad1.left_stick_y, 3) + Math.pow(gamepad1.right_stick_x, 3)) - Math.pow(gamepad1.left_stick_x, 3));
        frontLeft.setPower((Math.pow(gamepad1.left_stick_y, 3) - Math.pow(gamepad1.right_stick_x, 3)) - Math.pow(gamepad1.left_stick_x, 3));
        backLeft.setPower((Math.pow(gamepad1.left_stick_y, 3) - Math.pow(gamepad1.right_stick_x, 3)) + Math.pow(gamepad1.left_stick_x, 3));
    }
}
