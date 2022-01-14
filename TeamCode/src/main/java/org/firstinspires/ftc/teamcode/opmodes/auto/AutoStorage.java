package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TsePipeline;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
public class AutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;

    public double bottomRectWidthPercentage = 0.75;
    public double bottomRectHeightPercentage = 0.41;
    public double middleRectWidthPercentage = 0.37;
    public double middleRectHeightPercentage = 0.315;
    public double topRectWidthPercentage = 0.08;
    public double topRectHeightPercentage = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        TsePipeline.topRectWidthPercentage = topRectWidthPercentage;
        TsePipeline.topRectHeightPercentage = topRectHeightPercentage;
        TsePipeline.middleRectWidthPercentage = middleRectWidthPercentage;
        TsePipeline.middleRectHeightPercentage = middleRectHeightPercentage;
        TsePipeline.bottomRectWidthPercentage = bottomRectWidthPercentage;
        TsePipeline.bottomRectHeightPercentage = bottomRectHeightPercentage;

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoCarousel carousel = new AutoCarousel(hardwareMap);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true);
        final int[] height = {-1};

        final Pose2d initial = new Pose2d(-40, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, 55 * multiplier));
        builder.splineToLinearHeading(new Pose2d(-20, 45 * multiplier, Math.toRadians(100)),
                Math.toRadians(-110));
        builder.addTemporalMarker(() -> lift.setPosition(getPosition(height[0])));
        builder.waitSeconds(4);
        builder.addTemporalMarker(() -> lift.setPosition(AutoLift.Positions.INTAKING));
//        builder.waitSeconds(1);
        builder.lineTo(new Vector2d(-19, 45 * multiplier));
        builder.lineToLinearHeading(new Pose2d(-59, 57, Math.toRadians(240)));
        builder.addTemporalMarker(carousel::on);
        builder.waitSeconds(4);
        builder.addTemporalMarker(carousel::off);
        builder.lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(90)));
//        builder.waitSeconds(10);
//        builder.addTemporalMarker(() -> {
//            // TODO FIRE TAPE MEASURE
//        });
//        builder.waitSeconds(3);

        TrajectorySequence trajSeq = builder.build();

        Thread thread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
            }
        });

        waitForStart();
        thread.start();

        height[0] = detector.run();
        telemetry.addData("height", height[0]);
        telemetry.update();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
        drive.followTrajectorySequenceAsync(null);
        drive.update();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public AutoLift.Positions getPosition(int input) {
        return input == 1 ? AutoLift.Positions.BOTTOM :
                input == 2 ? AutoLift.Positions.MIDDLE : AutoLift.Positions.TOP;
    }
}
