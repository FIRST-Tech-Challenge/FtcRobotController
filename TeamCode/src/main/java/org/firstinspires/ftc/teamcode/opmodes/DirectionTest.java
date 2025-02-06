package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DirectionTest")
public class DirectionTest extends LinearOpMode {

    public static class Params {
        public double speedMult      = 1;
        public double turnMult       = 1;

        public double backMotorMult  = 1;
        public double frontMotorMult = 1;

        public double kP             = 2;
        public double kI             = 0.1;
        public double kD             = 0.2;

        public double power          = 1;
    }
    public static PrimaryOpMode.Params PARAMS = new PrimaryOpMode.Params();

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor         = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor          = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor        = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor         = hardwareMap.dcMotor.get("backRight");
        waitForStart();

        if (opModeIsActive()) {
            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");

            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            imu.resetYaw();

            double botHeading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            while (opModeIsActive()) {
                double y  = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x  = 1.1 * -gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                botHeading = unwrapAngle(botHeading, currentHeading); // Use unwrapping here

                double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                rotX *= PARAMS.speedMult;
                rotY *= PARAMS.speedMult;
                rx   *= PARAMS.turnMult;

                double denominator     = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower  = PARAMS.frontMotorMult * (rotY + rotX + rx) / denominator;
                double backLeftPower   = PARAMS.backMotorMult * (rotY - rotX + rx) / denominator;
                double frontRightPower = PARAMS.frontMotorMult * (rotY - rotX - rx) / denominator;
                double backRightPower  = PARAMS.backMotorMult *(rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
        }
    }

    private double unwrapAngle(double previousAngle, double currentAngle) {
        double delta = currentAngle - previousAngle;
        if (delta > Math.PI) {
            delta -= 2 * Math.PI;
        } else if (delta < -Math.PI) {
            delta += 2 * Math.PI;
        }
        return previousAngle + delta;
    }
}
