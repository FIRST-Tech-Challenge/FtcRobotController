package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplateCV;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel WH Park Autonomous🟦")
public class BlueCarouselAutonomousWHPark extends AutoObjDetectionTemplateCV {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));
    final static Pose2d warehouseCrossPos = new Pose2d(11, 46, 0);

    final static BlinkinPattern defaultColor = BlinkinPattern.BLUE;

    public static TrajectorySequence toEnd(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                //Park
                .back(2)
                .lineToLinearHeading(warehouseCrossPos.plus(new Pose2d(5, 23, Math.toRadians(20))))
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        initAll();

        leds.setPattern(defaultColor);

        drive.setPoseEstimate(startPos);

        // From
        final Trajectory toGoal = BlueCarouselAutonomous.ToGoalTraj(drive, startPos);

        final TrajectorySequence toSpinner = BlueCarouselAutonomous.ToSpinner(drive, toGoal.end());

        final TrajectorySequence toPark = toEnd(drive, toSpinner.end());

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

            //Drop off item
            {
                drive.turnTo(dropOffPos.getHeading());

                switch (pos) {
                    case NotSeen:
                    case Right:
                        slide.setTargetLevel(HeightLevel.TopLevel);
                        break;

                    case Center:
                        slide.setTargetLevel(HeightLevel.MiddleLevel);
                        break;

                    case Left:
                        slide.setTargetLevel(HeightLevel.BottomLevel);
                        break;
                }

                //Wait for the slide to reach position
                slide.waitOn();

                outtake.open();
                Thread.sleep(2000);
                outtake.close();
                slide.setTargetLevel(HeightLevel.Down);
            }

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
