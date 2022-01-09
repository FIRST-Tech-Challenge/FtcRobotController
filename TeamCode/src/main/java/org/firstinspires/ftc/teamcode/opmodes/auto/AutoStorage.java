package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
public class AutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoCarousel carousel = new AutoCarousel(hardwareMap);
//        TseDetector detector = new TseDetector(hardwareMap, "webcam", true);
        AtomicInteger height = new AtomicInteger();

        final Pose2d initial = new Pose2d(-40, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, 55 * multiplier));
        builder.splineToLinearHeading(new Pose2d(-20, 42 * multiplier, Math.toRadians(100)),
                Math.toRadians(-110));
        builder.addDisplacementMarker(() -> {
            // TODO LIFT UP
        });
        builder.waitSeconds(2);
        builder.addDisplacementMarker(() -> {
            // TODO LIFT DOWN
        });
        builder.waitSeconds(2);
        builder.lineTo(new Vector2d(-19, 45 * multiplier));
        builder.lineToLinearHeading(new Pose2d(-58, 56.5, Math.toRadians(240)));
        //builder.addDisplacementMarker(carousel::on);
        builder.waitSeconds(3);
        //builder.addDisplacementMarker(carousel::off);
        builder.lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(90)));
        builder.waitSeconds(10);
        builder.addDisplacementMarker(() -> {
            // TODO FIRE TAPE MEASURE
        });
        builder.waitSeconds(3);

        TrajectorySequence trajSeq = builder.build();

//        Thread detectorThread = new Thread(() -> {
//            height.set(detector.run());
//        });

        waitForStart();

//        detectorThread.start();

        if (!isStopRequested()) {
            carousel.on();
            drive.followTrajectorySequence(trajSeq);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
