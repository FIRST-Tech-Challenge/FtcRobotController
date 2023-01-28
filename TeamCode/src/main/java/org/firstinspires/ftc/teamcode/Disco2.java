package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Disco2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor armMotor1 = hardwareMap.dcMotor.get("arm1");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("arm2");

        Servo intake = hardwareMap.servo.get("intake");
        Servo axon1 = hardwareMap.servo.get("axon1");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double armPower = gamepad2.left_stick_y;
            boolean dpad2Up = gamepad2.dpad_up;
            boolean dpad2Down = gamepad2.dpad_down;

            double intakePos = gamepad2.right_stick_y;

            double axonPos = axon1.getPosition();
            double nextAxonPos = axonPos;

            if (dpad2Up) {
                intake.setPosition(0.0);
            }
            if (dpad2Down) {
                intake.setPosition(1.0);
            }
            double leftTriggerAxon = gamepad2.left_trigger;
            double rightTriggerAxon = gamepad2.right_trigger;
            double axonArmPower;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            armMotor1.setPower(armPower);
            armMotor2.setPower(-armPower);

            if (intakePos > 0.75) {
                nextAxonPos = axonPos + 0.02;
                sleep(10);
            }
            if (intakePos < -0.75) {
                nextAxonPos = axonPos - 0.1;
            }

            axon1.setPosition(nextAxonPos);

//            axon1.setPosition(intakePos);

            telemetry.addData("axon 1", axonPos);
            telemetry.update();
        }
    }
}