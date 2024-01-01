package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="Test for Encoders")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(hardwareMap);

        // resets encoder positions for use
        hardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Encoder leftEncoder = new Encoder(hardware.backRight);
        Encoder rightEncoder = new Encoder(hardware.frontLeft);
        Encoder frontEncoder = new Encoder(hardware.frontRight);

        Telemetry telemetries = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Resets motors for normal functionality.
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()) {


            telemetries.addLine(String.valueOf(leftEncoder.getCurrentPosition()));
            telemetries.addLine(String.valueOf(rightEncoder.getCurrentPosition()));
            telemetries.addLine(String.valueOf(frontEncoder.getCurrentPosition()));

//            hardware.frontLeft.setPower(y + x + rx);
//            hardware.frontRight.setPower(y - x - rx);
//            hardware.backLeft.setPower(y - x + rx);
//            hardware.backRight.setPower(y + x - rx);

            hardware.frontLeft.setPower(gamepad1.left_stick_y);
            hardware.frontRight.setPower(gamepad1.left_stick_x);
            hardware.backLeft.setPower(gamepad1.right_stick_x);
            hardware.backRight.setPower(gamepad1.right_stick_y);

            telemetries.addLine("Front Left Motor  (l1 y):  " + String.valueOf(hardware.frontLeft.getPower()));
            telemetries.addLine("Front Right Motor (l1 x):  " + String.valueOf(hardware.frontRight.getPower()));
            telemetries.addLine("Back Left Motor   (r1 x):  " + String.valueOf(hardware.backLeft.getPower()));
            telemetries.addLine("Back Right Motor  (r1 y):  " + String.valueOf(hardware.backRight.getPower()));
            telemetries.addData("Test Thingy 1  ", hardware.backRight.getPower());
            telemetries.update();
        }
    }
}
