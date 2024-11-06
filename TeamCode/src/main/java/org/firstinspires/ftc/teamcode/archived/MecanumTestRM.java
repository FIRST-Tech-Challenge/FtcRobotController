package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class MecanumTestRM extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hello",", Team RoboActive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double driveTrainSpeed = 0.1;
        // double armMotorSpeed = 0.1;
        double leftsticky = 0.1;
        double leftstickx = 0.1;
        double reverseRight = 0;
        double forwardleft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            if(gamepad1.b) {
                // Stop
                backLeft.setPower(0);
                backRight.setPower(0);
            } if (gamepad1.left_stick_y == -1) {
                // Move forward
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running forward at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_y == 1) {
                // Move backward
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running backward at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_x == -1) {
                //Strafe Left
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_x == 1) {
                //Strafe Right
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_trigger == 1) {
                //Turn Left
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
            }if (gamepad1.right_trigger == 1) {
                //Turn Right
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
            } if (gamepad1.right_bumper) {
                // Increase power by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_bumper) {
                // Reduce power by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (forwardleft >= 1.2) {
                //Strafe Diagonal Forward Left
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
            } if (forwardRight >= 1.2) {
                //Strafe Diagonal Forward Right
                frontLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
            } if (reverseLeft >= 1.2) {
                //Strafe Diagonal Reverse Left
                frontLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
            } if (reverseRight >= 1.2) {
                //Strafe Diagonal Reverse Right
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
            }

            reverseLeft = -leftstickx + leftsticky;
            forwardRight = leftstickx + -leftsticky;
            forwardleft = -leftstickx + -leftsticky;
            reverseRight = leftstickx + leftsticky;
            leftsticky = gamepad1.left_stick_y;
            leftstickx = gamepad1.left_stick_x;
        }
    }
}