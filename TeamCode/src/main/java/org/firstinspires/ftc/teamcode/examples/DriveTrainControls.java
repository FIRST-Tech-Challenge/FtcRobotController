package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

// @TeleOp
@Disabled
public class DriveTrainControls extends LinearOpMode {

    private DcMotorEx armMotor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private void move(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        telemetry.addData("Motor running forward at", frontLeftPower);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hello",", Team KryptoDragons");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double motorStop = 0;
        double driveTrainSpeed = 0.1;
        double leftsticky = 0.1;
        double leftstickx = 0.1;
        double reverseRight = 0;
        double forwardLeft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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

            if (gamepad1.left_stick_y == -1) {
                // Move forward
                move(driveTrainSpeed,driveTrainSpeed,driveTrainSpeed,driveTrainSpeed);
            } if (gamepad1.left_stick_y == 1) {
                // Move backward
                move(-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed);
            } if (gamepad1.left_stick_x == -1) {
                //Strafe Left
                move(-driveTrainSpeed,driveTrainSpeed,driveTrainSpeed,-driveTrainSpeed);
            } if (gamepad1.left_stick_x == 1) {
                //Strafe Right
                move(driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_trigger == 1) {
                //Turn Left
                move(-driveTrainSpeed,driveTrainSpeed,-driveTrainSpeed,driveTrainSpeed);
            }if (gamepad1.right_trigger == 1) {
                //Turn Right
                move(driveTrainSpeed,-driveTrainSpeed,driveTrainSpeed,-driveTrainSpeed);
            } if (gamepad1.right_bumper) {
                // Increase power of driveTrain by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_bumper) {
                // Reduce power of driveTrain by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (forwardLeft > 1.25) {
                //Strafe Diagonal Forward Left
                move(motorStop,driveTrainSpeed,driveTrainSpeed,motorStop);
            } if (forwardRight > 1.25) {
                //Strafe Diagonal Forward Right
                move(driveTrainSpeed,motorStop,motorStop,driveTrainSpeed);
            } if (reverseLeft > 1.25) {
                //Strafe Diagonal Reverse Left
                move(-driveTrainSpeed,motorStop,motorStop,-driveTrainSpeed);
            } if (reverseRight >= 1.25) {
                //Strafe Diagonal Reverse Right
                move(motorStop,-driveTrainSpeed,-driveTrainSpeed,motorStop);
            }

            reverseLeft = -leftstickx + leftsticky;
            forwardRight = leftstickx + -leftsticky;
            forwardLeft = -leftstickx + -leftsticky;
            reverseRight = leftstickx + leftsticky;
            leftsticky = gamepad1.left_stick_y;
            leftstickx = gamepad1.left_stick_x;
        }
    }
}