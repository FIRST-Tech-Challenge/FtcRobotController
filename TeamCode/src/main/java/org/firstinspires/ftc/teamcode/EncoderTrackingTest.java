package org.firstinspires.ftc.teamcode;

import static java.lang.StrictMath.PI;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;

import java.util.List;

@TeleOp
public class EncoderTrackingTest extends LinearOpMode {
    private double ticksPerRotation;
    private double radiusInches = 0.70;
    private double trackWidth; // distance between two parallel encoders
    private double forwardOffset;

    private double tick2inch(int ticks) {
        return (ticks / ticksPerRotation) * 2 * PI * radiusInches;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(hardwareMap);
        ticksPerRotation = hardware.getEncoderTicksPerRevolution();
        radiusInches = hardware.getEncoderWheelRadius();
        trackWidth = hardware.getTrackWidth();
        forwardOffset = hardware.getForwardOffset();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(consumer -> {
            consumer.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        });

        EncoderTracking encTrack = new EncoderTracking(hardware);

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        final double pow = 0.25;
        final double duration = 4.0;
        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        hardware.frontLeft.setPower(pow);
//        hardware.frontRight.setPower(pow);
//        hardware.backLeft.setPower(pow);
//        hardware.backRight.setPower(pow);
        while (opModeIsActive()) {
//            if (timer.time() > duration) {
//                hardware.frontLeft.setPower(0);
//                hardware.frontRight.setPower(0);
//                hardware.backLeft.setPower(0);
//                hardware.backRight.setPower(0);
//            }
            encTrack.step();
            Pose pose = encTrack.getPose();

//            telemetry.addData("enc:left", currentLeft);
//            telemetry.addData("enc:center", currentCenter);
//            telemetry.addData("enc:right", currentRight);
//            telemetry.addLine();
            telemetry.addData("x", pose.x());
            telemetry.addData("y", pose.y());
            telemetry.addData("heading deg", pose.heading() * 180 / PI);
            telemetry.update();
        }
    }
}
