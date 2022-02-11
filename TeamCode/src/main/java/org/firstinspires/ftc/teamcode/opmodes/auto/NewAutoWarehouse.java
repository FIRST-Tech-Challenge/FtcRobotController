package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.util.StayInPosition.stayInPose;
import static org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight.getPosition;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.TimedEvent;
import org.firstinspires.ftc.teamcode.opmodes.util.WallSmash;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class NewAutoWarehouse extends LinearOpMode {
    public int multiplier = 1;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        int height;
        double nextToWall = 70 - inchesToCoordinate(5.8D);

        MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry);

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoIntake intake = new AutoIntake(hardwareMap, eventThread);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        final Pose2d initial = new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);
        final Pose2d liftPosition = new Pose2d(-2, 43.5 * multiplier,
                Math.toRadians(85 * multiplier));

        ElapsedTime toolTimer = new ElapsedTime();
        ElapsedTime wallSmashTimer = new ElapsedTime();

        // Part 1: drive to alliance shipping hub
        final TrajectorySequence part1 = drive.trajectorySequenceBuilder(initial)
                .lineTo(new Vector2d(-3, 58 * multiplier))
                .lineToLinearHeading(liftPosition)
                .build();

        // where the robot **should** be after you intake
        final Pose2d intakeReturnPoint = new Pose2d(40, (nextToWall) * multiplier,
                0);

        // part 2: go to warehouse
        final TrajectorySequence part2 = drive.trajectorySequenceBuilder(liftPosition)
                .lineToLinearHeading(new Pose2d(0, (nextToWall) * multiplier))
                .addDisplacementMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, -0.2 * multiplier, 0)))
                .lineTo(intakeReturnPoint.vec())
                .build();


        // part 3: move back to Alliance Shipping hub. then you can go back to part 2 as needed.
        final TrajectorySequence part3 = drive.trajectorySequenceBuilder(intakeReturnPoint)
                .lineTo(new Vector2d(-3, nextToWall * multiplier))
                .lineToLinearHeading(liftPosition)
                .build();

        final boolean[] liftUpdated = {false};
        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
                liftUpdated[0] = true;
            }
        });

        waitForStart();
        liftThread.start();
        eventThread.start();

        intake.lightsOff();
        height = detector.run();
        intake.lightsOn();
        goodTelemetry.addData("height", height);
        goodTelemetry.update();

        drive.followTrajectorySequenceAsync(part1);
        updateLoop(drive);
        liftUpdated[0] = false;
        lift.setPosition(getPosition(height));
        toolTimer.reset();
        while (toolTimer.seconds() < 3) {
            if (isStopRequested()) {
                return;
            }
            stayInPose(drive, part1.end());
        }

        drive.followTrajectorySequenceAsync(part2);
        updateLoop(drive);
        if (isStopRequested()) return;

        WallSmash.smashIntoWall(drive, multiplier, 500);

        intake(toolTimer, drive, intake);

        WallSmash.smashIntoWall(drive, multiplier, 500);

        drive.followTrajectorySequenceAsync(part3);
        updateLoop(drive);
        if (isStopRequested()) return;

        lift.setPosition(AutoLift.Positions.TOP);
        toolTimer.reset();
        while (toolTimer.seconds() < 3) {
            if (isStopRequested()) {
                return;
            }
            stayInPose(drive, part3.end());
        }

        drive.followTrajectorySequenceAsync(part2);
        updateLoop(drive);
        intake(toolTimer, drive, intake);
        while (!isStopRequested()) {
            stayInPose(drive, part2.end());
        }
    }

    public void intake(ElapsedTime timer, SampleMecanumDrive drive, AutoIntake intake) {
        // intake a block
        intake.backward();
        drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
        while (intake.noObject()) {
            if (isStopRequested()) {
                return;
            }
        }
        timer.reset();
        while (timer.milliseconds() < 250) {}
        intake.stop();
        intake.forward();
        timer.reset();
        while (timer.milliseconds() < 100) {}
        intake.stop();
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }

    public void updateLoop(SampleMecanumDrive drive) {
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }
}
