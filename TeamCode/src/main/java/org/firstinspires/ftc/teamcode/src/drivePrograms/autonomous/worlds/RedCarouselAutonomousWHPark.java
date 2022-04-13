package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟥Red Carousel WH Park Autonomous🟥")
public class RedCarouselAutonomousWHPark extends AutonomousTemplate {
    static final Pose2d startPos = new Pose2d(-40, -65, Math.toRadians(0));
    static final Pose2d dropOffPos = new Pose2d(-33, -25, Math.toRadians(180));
    static final Pose2d parkPos = new Pose2d(-60, -35.5, Math.toRadians(270));
    static final Pose2d carouselSpinPos = new Pose2d(-65, -54, Math.toRadians(270));

    @Override
    public void opModeMain() throws InterruptedException {
        this.initLinearSlide();
        this.initOdometryServos();
        this.initLEDS();
        this.initSpinner();
        podServos.lower();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(12, -15)).vec())
                // Cross Box
                .splineToConstantHeading(parkPos.plus(new Pose2d(12, 15, 0)).vec(), 0)
                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(6, -4, Math.toRadians(-16))), 0)

                .build();

        final TrajectorySequence toSpinner = drive.trajectorySequenceBuilder(toGoal.end())
                // Cross Box
                .lineToLinearHeading(new Pose2d(parkPos.getX() + 5, parkPos.getY() + 15, Math.toRadians(270)))

                //.setConstraints((v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 20)
                // To Carousel Spinner
                .lineTo(carouselSpinPos.vec().plus(new Vector2d(5)))
                .build();

        final Trajectory toPark = drive.trajectoryBuilder(toSpinner.end())
                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(5, -2)))
                .build();

        telemetry.addData("Setup", "Finished");
        telemetry.update();
        waitForStart();


        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());
            slide.setTargetLevel(HeightLevel.TopLevel);


            Thread.sleep(1000);

            slide.setTargetLevel(HeightLevel.BottomLevel);

            Thread.sleep(1000);

            drive.followTrajectorySequence(toSpinner);

            spinner.spinOffRedDuck();

            //drive.followTrajectory(toPark);


        }
    }

}
