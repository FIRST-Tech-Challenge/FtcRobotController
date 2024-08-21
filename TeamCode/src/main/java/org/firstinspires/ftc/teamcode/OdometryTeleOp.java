package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.PoseFromToProcessor;

import java.util.List;

@TeleOp
public class OdometryTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(consumer -> {
            consumer.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        });

        DcMotor frontLeft = hardware.frontLeft;
        DcMotor backLeft = hardware.backLeft;
        DcMotor frontRight = hardware.frontRight;
        DcMotor backRight = hardware.backRight;

        EncoderTracking encTrack = new EncoderTracking(hardware);
        PoseFromToProcessor pftp = new PoseFromToProcessor(Pose.ORIGIN);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            encTrack.step();

            Pose p = encTrack.getPose();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            double diff = (frontLeftPower + backLeftPower) - (frontRightPower + backRightPower);
            pftp.update(diff, p);

            telemetry.addData("x", p.x());
            telemetry.addData("y", p.y());
            telemetry.addData("heading (rad)", p.heading());
            telemetry.addData("heading (deg)", p.heading() * 180 / PI);
            telemetry.update();
        }
    }
}
