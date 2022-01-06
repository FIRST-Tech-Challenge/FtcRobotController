/*
// This is a simple program which we will use to test values, driving styles, and other things. This is not our final version of the TeleOp but this will definitely contribute to it.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class chassisTest extends LinearOpMode {

    //Motor initialization
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

        while (opModeIsActive()) {

            //defining driving variables.
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            double cranePower;
            boolean armToggle;
            double carouselPower;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;

            cranePower = gamepad2.left_stick_y;
            armToggle = gamepad2.x;
            carouselPower = gamepad2.left_trigger;

            //making motors run.
            //strafing
            if (strafeLeft) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
            } else if (strafeRight) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
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

            crane.setPower(cranePower);

            if (armToggle){
                arm.setPosition(0);
            }else{
                arm.setPosition(1);
            }
            carousel.setPower(carouselPower);

            telemetry.addData("Elevator Trim", cranePower);
            System.out.println("servo position "+ arm.getPosition());
            telemetry.update();
        }
    }
}
