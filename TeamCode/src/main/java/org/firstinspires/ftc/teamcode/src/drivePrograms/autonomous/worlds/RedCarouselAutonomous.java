package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Config
@Autonomous(name = "Red Carousel Auton")
public class RedCarouselAutonomous extends GenericOpModeTemplate {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-12, -38, Math.toRadians(270));
    public static Pose2d carouselSpinPos = new Pose2d(-61, -51, Math.toRadians(270));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(270));


    @Override
    public void opModeMain() throws InterruptedException {
        this.initOdometryServos();
        podServos.lower();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);

        final TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(dropOffPos)
                .waitSeconds(1)
                // replace wait with a displacement marker
                .splineToLinearHeading(carouselSpinPos, Math.toRadians(180))
                .waitSeconds(1)
                // replace wait with a displacement marker
                .lineToLinearHeading(parkPos)
                .build();

        final Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(dropOffPos)

                .build();
        final Trajectory traj2 = drive.trajectoryBuilder(dropOffPos)
                .splineToLinearHeading(carouselSpinPos, Math.toRadians(180))
                .build();
        final Trajectory traj3 = drive.trajectoryBuilder(carouselSpinPos)
                .lineToLinearHeading(parkPos)
                .build();

        waitForStart();

        if (!isStopRequested()) {
            //drive.followTrajectorySequence(trajSeq);
            drive.followTrajectory(traj1);
            Thread.sleep(1000);
            drive.followTrajectory(traj2);
            Thread.sleep(1000);
            drive.followTrajectory(traj3);
        }

        telemetry.addData("Cumulative Error (in/sec)", TrajectorySequenceRunner.totalError/1000);
        telemetry.addData("Average Error (in)", TrajectorySequenceRunner.totalError / TrajectorySequenceRunner.totalTime);

        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) Thread.yield();
    }

}
