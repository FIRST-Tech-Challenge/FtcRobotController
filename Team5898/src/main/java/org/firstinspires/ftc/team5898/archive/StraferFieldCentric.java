package org.firstinspires.ftc.team5898.archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This particular OpMode executes a field centric Teleop for a mecanum robot
 * The code is structured as a LinearOpMode
 *
 * In this mode, the left stick moves the robot relative to the field.
 * The Right stick rotates the robot left and right.
 * The Logitech button resets the robot's heading.
 *
 */

@Disabled
@TeleOp(name="Strafer - Field Centric", group="Starter Code")
public class StraferFieldCentric extends LinearOpMode {

    IMU imu;

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("RL");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("RR");

        // These are the extra moving parts
        DcMotor motorArmTilt = hardwareMap.dcMotor.get("Arm");
        DcMotor motorBeltDrive = hardwareMap.dcMotor.get("Belt");
        Servo servoClaw = hardwareMap.servo.get("Claw");
        Servo servoWrist = hardwareMap.servo.get("Wrist");
        double wristPos = 1.0;
        servoWrist.setPosition(wristPos);
        sleep(1000);
        double CLAW_CLOSE = 0.27;
        servoClaw.setPosition(CLAW_CLOSE);//close

        // funny drive

        // Reverse the left side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        motorBeltDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBeltDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBeltDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmTilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers or options on PS4-style controllers.
            if (gamepad1.guide) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            int slidePos = motorBeltDrive.getCurrentPosition();
            int tiltPos = motorArmTilt.getCurrentPosition();
            telemetry.addData("Current slide position", slidePos);
            telemetry.addData("Current arm position", tiltPos);
            telemetry.update();

            if (gamepad2.dpad_up && slidePos <= 2300)
            {
                motorBeltDrive.setPower(.5);
            }
            else if (gamepad2.dpad_down && slidePos >= 30)
            {
                motorBeltDrive.setPower(-.5);
            }
            else
            {
                motorBeltDrive.setPower(0);
            }

            if (gamepad2.left_bumper)
            {
                motorArmTilt.setPower(-.75);
            }
            else if (gamepad2.right_bumper)
            {
                motorArmTilt.setPower(.75);
            }
            else if (!gamepad2.right_bumper && !gamepad1.left_bumper){
                motorArmTilt.setPower(0);
            }

            if (gamepad2.y)
            {
                wristPos -= .001;
                servoWrist.setPosition(wristPos);
            }
            else if (gamepad2.a)
            {
                wristPos += .001;
                servoWrist.setPosition(wristPos);
            }

            if (gamepad1.left_bumper)
            {
                servoClaw.setPosition(.4);//open
            }
            else if (gamepad1.right_bumper)
            {
                servoClaw.setPosition(CLAW_CLOSE);//close
            }
        }
    }
}
