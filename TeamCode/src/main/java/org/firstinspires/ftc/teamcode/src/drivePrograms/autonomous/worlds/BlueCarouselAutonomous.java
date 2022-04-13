package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplateCV;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel Autonomous🟦")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplateCV {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));

    final static BlinkinPattern defaultColor = BlinkinPattern.BLUE;

    @Override
    public void opModeMain() throws InterruptedException {
        initAll();
        this.switchWebcam();

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

        final Trajectory toPark = drive.trajectoryBuilder(toSpinner.end())
                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(8, -1)))
                .build();

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        leds.setPattern(BlinkinPattern.WHITE);
        BarcodePositions pos;
        do {
            pos = this.findPositionOfMarker();
            telemetry.addData("Pos", pos);
            telemetry.update();

        } while (!opModeIsActive() && !isStarted());
        leds.setPattern(defaultColor);


        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());
            slide.setTargetLevel(HeightLevel.TopLevel);


            Thread.sleep(1000);

            switch (pos) {
                case NotSeen:
                case Right:
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    break;

                case Center:
                    slide.setTargetLevel(HeightLevel.MiddleLevel);

                case Left:
                    slide.setTargetLevel(HeightLevel.BottomLevel);
            }


            Thread.sleep(1000);

            drive.followTrajectorySequence(toSpinner);

            spinner.spinOffBlueDuck();

            drive.followTrajectory(toPark);


        }
    }

}
