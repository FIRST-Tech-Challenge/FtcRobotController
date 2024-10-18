package org.firstinspires.ftc.teamcode.vrsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lesson5", group = "Tutorial")
public class Lesson5 extends LinearOpMode {
    // create DcMotor objects
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    @Override
    public void runOpMode() {
        // initialize hardware variables
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Set motor direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // set mode to STOP_AND_RESET_ENCODER to set position to zero
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode to RUN_TO_POSITION
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set target position
        frontLeft.setTargetPosition(4500);

        // define variable for easier programming
        double on = 0.9;

        waitForStart();

        // when started
        if (opModeIsActive()) {

            // loop until reach target position
            while (frontLeft.getCurrentPosition() < frontLeft.getTargetPosition()) {
                frontRight.setPower(on);
                frontLeft.setPower(on);
                backRight.setPower(on);
                backLeft.setPower(on);
                // displays encoder position
                telemetry.addData("Position", frontLeft.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }

            // turn left
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            sleep(100);

            // stop and reset encoder
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // turn left
            frontRight.setPower(1);
            frontLeft.setPower(-1);
            backRight.setPower(1);
            backLeft.setPower(-1);
            sleep(210);

            // set mode to RUN_TO_POSITION
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // set target position
            frontLeft.setTargetPosition(4500);

            // loop until target position
            while (frontLeft.getCurrentPosition() < frontLeft.getTargetPosition()) {
                frontRight.setPower(on);
                frontLeft.setPower(on);
                backRight.setPower(on);
                backLeft.setPower(on);
                // display encoder position
                telemetry.addData("Position", frontLeft.getCurrentPosition());
                telemetry.update();
                sleep(50);
            }

            // stop car
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            sleep(2000);
        }
    }
}
