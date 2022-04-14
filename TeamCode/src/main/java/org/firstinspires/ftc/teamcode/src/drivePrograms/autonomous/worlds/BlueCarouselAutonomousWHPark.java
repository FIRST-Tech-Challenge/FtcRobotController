package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel WH Park Autonomous🟦")
public class BlueCarouselAutonomousWHPark extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d warehouseCrossPos = new Pose2d(11, 46, 0);

    public BlueCarouselAutonomousWHPark() {
        super(BlinkinPattern.BLUE);
    }

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

        final BarcodePositions pos = this.monitorMarkerWhileWaitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);

            this.dropOffItem(pos);

            drive.followTrajectorySequence(toSpinner);

            spinner.spinOffBlueDuck();

            drive.followTrajectorySequence(toPark);
            this.driveOverBarriers();


        }
    }

}
