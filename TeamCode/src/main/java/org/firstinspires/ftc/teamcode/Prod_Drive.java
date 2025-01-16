package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ProdDrive")
public class Prod_Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //mapping all the motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBackDrive");

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
       
        // This is the line that ends the init of the bot
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // These lines assign game-pad 1 joysticks to variables
            double ly = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double lx = gamepad1.left_stick_x;

            // This makes variables for the motor power and sets it based on some math
            // That takes the joystick x and y and does some things for motor power
            double denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(lx), 1);
            double frontLeftPower = (ly + rx + lx) / denominator;
            double backLeftPower = (ly - rx + lx) / denominator;
            double frontRightPower = (ly - rx - lx) / denominator;
            double backRightPower = (ly + rx - lx) / denominator;

            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);

            //GamePad B button
            if (gamepad1.b) {
              telemetry.addData("B is being pressed", gamepad1.b);
            }

            //GamePad A button
            if (gamepad1.a) {
              telemetry.addData("A is being pressed", gamepad1.a);
            }
            
            //GamePad X button
            if (gamepad1.x) {
              telemetry.addData("x is being pressed", gamepad1.x);
            }

            //GamePad Y button
            if (gamepad1.y) {
              telemetry.addData("Y is being pressed", gamepad1.y);
            }

            //GamePad right bumper
            if (gamepad1.right_bumper) {
              telemetry.addData("right bumper is being pressed", gamepad1.right_bumper);
            }

            //GamePad left bumper
            if (gamepad1.left_bumper) {
              telemetry.addData("left bumper is being pressed", gamepad1.left_bumper);
            }

            //GamePad right trigger
            if (gamepad1.right_trigger > 0) {
              telemetry.addData("right trigger is being pressed", gamepad1.right_trigger);
            }

            //GamePad left trigger
            if (gamepad1.left_trigger > 0) {
              telemetry.addData("left trigger is being pressed", gamepad1.left_trigger);
            }

            // Adds telemetry on the control hub to check stick positions
            telemetry.addData("Gamepad X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad Y", gamepad1.left_stick_y);

            // Sends it to the control hub
            telemetry.update();
        }
    }
}
