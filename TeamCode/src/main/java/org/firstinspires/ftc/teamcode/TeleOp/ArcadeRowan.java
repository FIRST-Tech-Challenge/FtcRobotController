package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ArcadeRowan extends LinearOpMode {

    //Initializing motor variables
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    DcMotor intake;
    DcMotor lift;

    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");

        //resetting encoder values for lift
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Waiting for Start button to be pressed
        waitForStart();

        //Looping while the opmode is running
        while (opModeIsActive()){
            //defining driving variables (throttle = moving)
            double throttle = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean strafeR = gamepad1.right_bumper;
            boolean strafeL = gamepad1.left_bumper;

            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            backLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backRight.setPower(throttle);

            //setting up strafing
            if(strafeR) {
                frontLeft.setPower(0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(0.75);
            } else if (strafeL) {
                frontLeft.setPower(-0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(0.75);
                backRight.setPower(-0.75);
            }

            //setting power for turning
            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);

            //setting power for intake
            if(gamepad2.a) {
                intake.setPower(1);
            } else if (gamepad2.b) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            //setting power for lift
            lift.setPower(-1 * gamepad2.right_stick_y);
            telemetry.addData("lift position", lift.getCurrentPosition());
            telemetry.update();

            if (lift.getCurrentPosition() <= 0) {
                if (gamepad2.right_stick_y < 0) {
                    lift.setPower(0);
                } else {
                    lift.setPower(-1 * gamepad2.right_stick_y);
                }
            } else if (gamepad2.right_stick_y >= 1000) {
                if (gamepad2.right_stick_y > 0) {
                    lift.setPower(0);
                } else {
                    lift.setPower(-1 * gamepad2.right_stick_y);
                }
            } else {
                lift.setPower(-1 * gamepad2.right_stick_y);
            }

        }
    }
}
