package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricCode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor FLM = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor BLM = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor FRM = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor BRM = hardwareMap.dcMotor.get("backRightMotor");

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double ls = gamepad1.left_stick_x;
            double rs = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = ls * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = ls * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rs), 1);
            double frontLeftPower = (rotY + rotX + rs) / denominator;
            double backLeftPower = (rotY - rotX + rs) / denominator;
            double frontRightPower = (rotY - rotX - rs) / denominator;
            double backRightPower = (rotY + rotX - rs) / denominator;

            FLM.setPower(frontLeftPower);
            BLM.setPower(backLeftPower);
            FRM.setPower(frontRightPower);
            BRM.setPower(backRightPower);
        }
    }
}
