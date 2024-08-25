package org.firstinspires.ftc.teamcode;

import static java.lang.StrictMath.PI;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Pose;

import java.util.List;

@TeleOp
public class EncoderTrackingTest extends LinearOpMode {
    private static final double ticksPerRotation = 8192.0;
    private static final double radiusInches = 0.70;
    private static final double trackWidth = 14 + 7 / 16.; // distance between two parallel encoders
    private static final double forwardOffset = -(6 + 3 / 4.);

    private double tick2inch(int ticks) {
        return (ticks / ticksPerRotation) * 2 * PI * radiusInches;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(hardwareMap);
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
        hardware.frontLeft.setPower(pow);
        hardware.frontRight.setPower(pow);
        hardware.backLeft.setPower(pow);
        hardware.backRight.setPower(pow);
        while (opModeIsActive()) {
            if (timer.time() > duration) {
                hardware.frontLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.backRight.setPower(0);
            }
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
