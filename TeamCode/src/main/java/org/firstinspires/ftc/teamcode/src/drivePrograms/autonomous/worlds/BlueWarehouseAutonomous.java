package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Warehouse Autonomous🟦")
public class BlueWarehouseAutonomous extends AutonomousTemplate {
    final static RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    final static Pose2d startPos = new Pose2d(7, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-12, 38, Math.toRadians(90));
    final static Pose2d whEntryPos = new Pose2d(20, 65, Math.toRadians(0));

    @Override
    public void opModeMain() throws InterruptedException {
        this.initLinearSlide();
        this.initOdometryServos();
        this.initLEDS();
        this.initSpinner();
        podServos.lower();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leds.setPattern(defaultColor);

        drive.setPoseEstimate(startPos);

        final TrajectorySequence startToHub = drive.trajectorySequenceBuilder(startPos)
                .strafeRight(5)
                .lineToSplineHeading(dropOffPos.plus(new Pose2d(2, 3, 0)))
                //This path should not be replicated for additional freight since it is inefficient.

                /* the following sequence is the path our robot takes to
                do cycles of freight. Because this would ideally be looped in our
                opmode, this section should be a pre-built Trajectory that is looped in Run()
                  */
                .waitSeconds(1)
                .build();

        final TrajectorySequence hubToWH = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(whEntryPos.plus(new Pose2d(0, 0, 0)), Math.toRadians(0))

                .forward(10)
                .waitSeconds(1)
                // this is where we will pick up additional freight
                .build();

        final TrajectorySequence WHToHub = drive.trajectorySequenceBuilder(hubToWH.end())
                .back(10)
                .setReversed(true)
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(1, 4, 0)), Math.toRadians(-90))
                .setReversed(false)
                .build();

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHub);
            drive.turnTo(dropOffPos.getHeading());

            for (int i = 0; i < 5; i++) {
                drive.followTrajectorySequence(hubToWH);
                drive.followTrajectorySequence(WHToHub);
                drive.turnTo(dropOffPos.getHeading());
            }

        }

    }
}
