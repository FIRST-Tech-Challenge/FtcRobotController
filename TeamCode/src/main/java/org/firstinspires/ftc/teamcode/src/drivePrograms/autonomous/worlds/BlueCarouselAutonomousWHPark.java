package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel WH Park Autonomous🟦")
public class BlueCarouselAutonomousWHPark extends AutonomousTemplate {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));
    final static Pose2d warehouseCrossPos = new Pose2d(11, 46, 0);

    final static BlinkinPattern defaultColor = BlinkinPattern.BLUE;

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

        // From
        final Trajectory toGoal = drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(20, 15, 0)).vec())
                // Cross Box
                .splineToConstantHeading(parkPos.plus(new Pose2d(25, -15, 0)).vec(), 0)
                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(6, 4, Math.toRadians(18))), 0)
                .build();

        final TrajectorySequence toSpinner = drive.trajectorySequenceBuilder(toGoal.end())
                // Cross Box
                .lineToLinearHeading(new Pose2d(parkPos.getX() + 15, parkPos.getY() - 15, Math.toRadians(95)))

                //.setConstraints((v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 20)
                // To Carousel Spinner
                .lineTo(carouselSpinPos.plus(new Pose2d(8, 4)).vec())
                .build();

        final TrajectorySequence toPark = drive.trajectorySequenceBuilder(toSpinner.end())
                //Park
                .back(2)
                .lineToLinearHeading(warehouseCrossPos.plus(new Pose2d(5, 23, Math.toRadians(20))))
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

            spinner.spinOffBlueDuck();

            //Thread.sleep(17000);

            drive.followTrajectorySequence(toPark);
            podServos.raise();
            Thread.sleep(1000);
            drive.setMotorPowers(1, 1, 1, 1);
            Thread.sleep(1000);
            drive.setMotorPowers(0, 0, 0, 0);


        }
    }

}
