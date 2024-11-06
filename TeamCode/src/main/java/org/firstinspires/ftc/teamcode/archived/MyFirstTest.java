package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class MyFirstTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hello",", Team KryptoDragons");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //double armMotorSpeed = 0;
        double driveTrainSpeed = 0.1;
        double leftsticky = 0.1;
        double leftstickx = 0.1;
        double reverseRight = 0;
        double forwardLeft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;
        //int armPosition = 0;

       // armMotor.setVelocity(250);
        //armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setTargetPosition(100);
        //armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        //armMotor.setVelocity(armMotorSpeed);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //armMotor.setVelocity(250);

            //telemetry.addData("is at target", !armMotor.isBusy());
            //telemetry.addData("armPosition",armPosition);
            //telemetry.update();

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
            } if (forwardLeft > 1.1) {
                //Strafe Diagonal Forward Left
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
            } if (forwardRight > 1.1) {
                //Strafe Diagonal Forward Right
                frontLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
            } if (reverseLeft > 1.1) {
                //Strafe Diagonal Reverse Left
                frontLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
            } if (reverseRight >= 1.1) {
                //Strafe Diagonal Reverse Right
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
            } if (gamepad1.left_bumper) {
                sleep(5);
                //armPosition = armPosition - 3;
                //telemetry.addData("armposition",armPosition);
                telemetry.update();
            } if (gamepad1.right_bumper) {
                sleep(5);
                //armPosition = armPosition + 3;
                //telemetry.addData("armposition",armPosition);
                telemetry.update();
            }
            //armMotor.setTargetPosition(armPosition);
            //armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);





            //telemetry.addData("armPosition",armPosition);
            telemetry.update();

            reverseLeft = -leftstickx + leftsticky;
            forwardRight = leftstickx + -leftsticky;
            forwardLeft = -leftstickx + -leftsticky;
            reverseRight = leftstickx + leftsticky;
            leftsticky = gamepad1.left_stick_y;
            leftstickx = gamepad1.left_stick_x;
        }
    }
}