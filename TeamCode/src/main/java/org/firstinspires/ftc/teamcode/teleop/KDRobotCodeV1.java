package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
// @TeleOp
@Disabled
public class KDRobotCodeV1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hello",", Team RoboActive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double driveTrainSpeed = 0.1;
        double armMotorSpeed = 0.1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            backLeft.setPower(0);
            backRight.setPower(0);
            armMotor.setPower(0);
            // gamepad1 will be used for Drive train
            // gamepad2 will be used for Arm and Intake.
            if(gamepad1.b) {
                // Stop
                backLeft.setPower(0);
                backRight.setPower(0);
            } else if (gamepad1.dpad_up) {
                // Move forward
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running forward at", driveTrainSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                // Move backward
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running backward at", driveTrainSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_left) {
                // Turn left
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_right) {
                // Turn right
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } else if (gamepad1.right_bumper) {
                // Increase power by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Incremented power to ", driveTrainSpeed);
                telemetry.update();
            }else if (gamepad1.left_bumper) {
                // Reduce power by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Reduced power to ", driveTrainSpeed);
                telemetry.update();
            } else if (gamepad2.b) {
                // Stop
                intakeServo.setPower(0);
                armMotor.setPower(0);
            } else if (this.gamepad2.left_trigger == 1.0) {
                // Intake OR Outake
                intakeServo.setPower(1);
                telemetry.addData("Servo position", "1");
                telemetry.update();
            } else if (this.gamepad2.right_trigger == 1.0) {
                // Intake OR Outake
                intakeServo.setPower(-1);
                telemetry.addData("Servo position", "-1");
                telemetry.update();
            } else if (gamepad2.dpad_up) {
                // Arm up
                armMotor.setPower(armMotorSpeed);
                telemetry.addData("armMotorSpeed", armMotorSpeed);
                telemetry.update();
            } else if (gamepad2.dpad_down) {
                // Arm down
                armMotor.setPower(-armMotorSpeed);
                telemetry.addData("armMotorSpeed", armMotorSpeed);
                telemetry.update();
            } else if (gamepad2.left_bumper) {
                // Reduce power -0.1
                sleep(250);
                armMotorSpeed = Range.clip(armMotorSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Reduced power to ", armMotorSpeed);
                telemetry.update();
            } else if (gamepad2.right_bumper) {
                // Increase power by 0.1
                sleep(250);
                armMotorSpeed = Range.clip(armMotorSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Incremented power to ", armMotorSpeed);
                telemetry.update();
            }
        }
    }
}