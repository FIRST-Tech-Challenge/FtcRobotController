package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleopRed")

public class TeleopRed extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft, backLeft, frontRight,backRight;
        CRServo carousel;

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        carousel = hardwareMap.get(CRServo.class, "carousel");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {
            //turn with right stick
            if (gamepad1.right_stick_x > 0.1) {
                telemetry.addData("positive", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.right_stick_x < -0.1) {
                telemetry.addData("negative", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.left_stick_x < -0.1 && gamepad1.left_stick_y > 0.1){
                //move UpLeft

            } else if (gamepad1.left_stick_y > 0.1){
                //move Up

            } else if (gamepad1.left_stick_x > 0.1 && gamepad1.left_stick_y > 0.1){
                //move UpRight

            } else if (gamepad1.left_stick_x > 0.1){
                //move Right

            } else if (gamepad1.left_stick_x > 0.1 && gamepad1.left_stick_y < -0.1){
                //move DownRight

            } else if (gamepad1.left_stick_y < -0.1){
                //move Down

            } else if (gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_y < -0.1){
                //move DownLeft

            } else if (gamepad1.left_stick_x < -0.1){
                //move Left

            }


            telemetry.update();

            turnDuck(carousel);
        }

    }

    protected void turnDuck(CRServo carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(0.9);
        } else {
            carousel.setPower(0);
        }
    }
}
