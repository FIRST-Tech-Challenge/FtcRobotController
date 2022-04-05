package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Red Carousel Auton")
public class RedCarouselAutonomous extends GenericOpModeTemplate {
     static final Pose2d startPos = new Pose2d(-34, -65, Math.toRadians(0));
     static final Pose2d dropOffPos = new Pose2d(-12, -38, Math.toRadians(270));
     static final Pose2d carouselSpinPos = new Pose2d(-61, -51, Math.toRadians(270));
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

        waitForStart();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
    }

}
