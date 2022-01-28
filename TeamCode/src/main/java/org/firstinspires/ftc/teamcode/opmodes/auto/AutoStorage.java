package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;
    protected boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoCarousel carousel = new AutoCarousel(hardwareMap, multiplier);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        final int[] height = {-1};

        final Pose2d initial = new Pose2d(-47, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, 55 * multiplier));
        builder.splineToLinearHeading(new Pose2d(-21, 42 * multiplier, Math.toRadians(100 * multiplier)),
                Math.toRadians(-110 * multiplier));
        builder.addTemporalMarker(() -> lift.setPosition(getPosition(height[0])));
        builder.waitSeconds(4);
        builder.lineTo(new Vector2d(-19, 50 * multiplier));
        builder.lineToLinearHeading(new Pose2d(-60, 58 * multiplier, Math.toRadians(240)));
        builder.addTemporalMarker(carousel::on);
        builder.waitSeconds(4);
        builder.addTemporalMarker(carousel::off);
        builder.lineToLinearHeading(new Pose2d(-60, 35 * multiplier, Math.toRadians(90)));
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
        height[0] = detector.run();

        thread.start();
        eventThread.start();

        telemetry.addData("height", height[0]);
        telemetry.update();
        drive.followTrajectorySequenceAsync(trajSeq);
        while (!isStopRequested()) {
            drive.update();
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        drive.followTrajectorySequenceAsync(null);
    }

    public AutoLift.Positions getPosition(int input) {
        return input == 1 ? AutoLift.Positions.BOTTOM :
                input == 2 ? AutoLift.Positions.MIDDLE : AutoLift.Positions.TOP;
    }
}
