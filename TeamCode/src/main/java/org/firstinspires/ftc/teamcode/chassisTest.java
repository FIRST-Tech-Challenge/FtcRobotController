/*
This code is simply testing the robot to make sure it works. We will make a final program later. In addition to this we will also be updating and changing this program.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class chassisTest extends LinearOpMode {

    //Motor initalization
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor carousel;
    private DcMotor crane;
    private Servo arm;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        crane = hardwareMap.get(DcMotor.class, "crane");
        arm = hardwareMap.get(Servo.class, "arm");
        //setting motor direction since some motors were backwards
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        double throttle = 0;
        boolean craneUp = false;
        boolean craneDown = false;
        while (opModeIsActive()) {

            //defining driving variables.
            double turn;
            double strafeValue;

            boolean buttonB;
            boolean armIn;
            boolean armOut;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeValue = gamepad1.left_stick_x;

            buttonB = gamepad2.b;
            craneUp = gamepad2.right_bumper;
            craneDown = gamepad2.left_bumper;
            armIn = gamepad2.dpad_left;
            armOut = gamepad2.dpad_right;

            //making motors run.
            //strafing
            if (strafeValue > 0.1) {
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            } else if (strafeValue < -0.1) {
                frontLeft.setPower(-strafeValue);
                frontRight.setPower(strafeValue);
                backLeft.setPower(strafeValue);
                backRight.setPower(-strafeValue);
            }
            //crane and carousel
            if (buttonB) {
                carousel.setPower(0.8);
            } else {
                carousel.setPower(0);
            }

            if (craneUp) {
                crane.setPower(0.8);
            } else if (craneDown) {
                crane.setPower(-0.8);
            } else {
                crane.setPower(0);
            }

            //arm open/close
            if (armIn) {
                arm.setPosition(0.65);
            } else if (armOut) {
                arm.setPosition(0.302);
            }

            //forward and backward movement
            frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            //turning
            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);
        }
        //telemetry for debugging
        System.out.println("Throttle " + throttle);
        System.out.println("Crane up " + craneUp);
        System.out.println("Crane down " + craneDown);
    }
}
