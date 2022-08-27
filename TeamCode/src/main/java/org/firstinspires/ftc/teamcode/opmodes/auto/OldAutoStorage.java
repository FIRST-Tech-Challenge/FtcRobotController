package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight.getPosition;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Disabled
public class OldAutoStorage extends LinearOpMode {
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

        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, multiplier == 1 ? 55 : -53));
        builder.splineToLinearHeading(new Pose2d(multiplier == 1 ? -21 : -20, multiplier == 1 ? 42 : -38, Math.toRadians(multiplier == 1 ? 100 : -95)),
                Math.toRadians(-110 * multiplier));
        builder.addTemporalMarker(() -> lift.setPosition(getPosition(height[0])));
        builder.waitSeconds(4);
        builder.lineTo(new Vector2d(-19, 50 * multiplier));
        builder.lineToLinearHeading(new Pose2d(multiplier == 1 ? -60 : -60.5, multiplier == 1 ? 58 : -58.5, Math.toRadians(multiplier == 1 ? 240 : 330)));
        builder.addTemporalMarker(carousel::on);
        builder.waitSeconds(4);
        builder.addTemporalMarker(carousel::off);
        builder.lineToLinearHeading(new Pose2d(-60, 35 * multiplier, Math.toRadians(90 * multiplier)));
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


}
