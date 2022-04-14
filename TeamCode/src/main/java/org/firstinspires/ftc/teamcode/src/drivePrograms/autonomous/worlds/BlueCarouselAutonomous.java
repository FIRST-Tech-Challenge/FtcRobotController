package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟦Blue Carousel Autonomous🟦")
public class BlueCarouselAutonomous extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(-34, 65, 0);
    final static Pose2d dropOffPos = new Pose2d(-27, 23.5, Math.toRadians(180));
    final static Pose2d carouselSpinPos = new Pose2d(-61, 51, Math.toRadians(90));
    final static Pose2d parkPos = new Pose2d(-60, 35.5, Math.toRadians(90));

    public BlueCarouselAutonomous() {
        super(BlinkinPattern.BLUE);
    }

    public static Trajectory ToGoalTraj(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectoryBuilder(startPos)
                // Side in
                .lineToConstantHeading(parkPos.plus(new Pose2d(20, 15, 0)).vec())
                // Cross Box
                .splineToConstantHeading(parkPos.plus(new Pose2d(25, -15, 0)).vec(), 0)
                //Approach Goal
                .splineToSplineHeading(dropOffPos.plus(new Pose2d(8, 4, Math.toRadians(18))), 0)
                .build();
    }

    public static TrajectorySequence ToSpinner(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                // Cross Box
                .lineToLinearHeading(new Pose2d(parkPos.getX() + 15, parkPos.getY() - 15, Math.toRadians(95)))

                //.setConstraints((v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 20)
                // To Carousel Spinner
                .lineTo(carouselSpinPos.plus(new Pose2d(8, 4)).vec())
                .build();
    }

    public static Trajectory ToEnd(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectoryBuilder(startPos)
                //Park
                .lineTo(parkPos.vec().plus(new Vector2d(8, -1)))
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

        final Trajectory toPark = ToEnd(drive, toSpinner.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = monitorMarkerWhileWaitForStart();


        waitForStart();

        if (!isStopRequested()) {

            drive.followTrajectory(toGoal);
            drive.turnTo(dropOffPos.getHeading());

            this.dropOffItem(pos);

            drive.followTrajectorySequence(toSpinner);

            spinner.spinOffBlueDuck();

            drive.followTrajectory(toPark);


        }
    }

}
