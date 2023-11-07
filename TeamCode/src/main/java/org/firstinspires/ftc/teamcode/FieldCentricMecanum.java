package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration, ok?
        DcMotor Left_Front = hardwareMap.dcMotor.get("Left_Front");
        DcMotor Left_Back = hardwareMap.dcMotor.get("Left_Back");
        DcMotor Right_Front = hardwareMap.dcMotor.get("Right_Front");
        DcMotor Right_Back = hardwareMap.dcMotor.get("Right_Back");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.75; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 0.75;
            double rx = -gamepad1.right_stick_x * 0.75;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double botHeadingRadian = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            // Rotate the movement direction counter to the bot's rotation
            double rotX = (x * Math.cos(-botHeadingRadian) - y * Math.sin(-botHeadingRadian)) / 2.0;
            double rotY =(x * Math.sin(-botHeadingRadian) + y * Math.cos(-botHeadingRadian)) / 2.0;

            // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Left_Front.setPower(frontLeftPower);
            Left_Back.setPower(backLeftPower);
            Right_Front.setPower(frontRightPower);
            Right_Back.setPower(backRightPower);
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("rotY", rotY);
            telemetry.addData("rotX", rotX);
            telemetry.addData("parameters", parameters);
            telemetry.addData("IMU", imu);
            telemetry.addData("denominator", denominator);
            telemetry.addData("botHeading", botHeading);
            telemetry.addData("botHeadingRadian", botHeadingRadian);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("time", time);
            telemetry.update();



        }
    }
}