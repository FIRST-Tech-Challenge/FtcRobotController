package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.util.StayInPosition.stayInPose;
import static org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight.getPosition;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.AutoLED;
import org.firstinspires.ftc.teamcode.opmodes.util.DelayStorage;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.opmodes.util.WallSmash;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoWarehouse extends LinearOpMode {
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
        final Pose2d initial = new Pose2d(0, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);
        final Pose2d liftPosition = isRed ? new Pose2d(-4, -43.3, Math.toRadians(-65)) :
                new Pose2d(-3, 45, Math.toRadians(65));

        ElapsedTime toolTimer = new ElapsedTime();

        // Part 1: drive to alliance shipping hub
        final TrajectorySequence part1 = drive.trajectorySequenceBuilder(initial)
                .lineTo(new Vector2d(-3, 58 * multiplier))
                .lineToLinearHeading(liftPosition)
                .build();

        // where the robot **should** be after you intake
        final Pose2d intakeReturnPoint = new Pose2d(40, (nextToWall) * multiplier,
                0);

        // part 2: go to warehouse
        final Trajectory part2 = drive.trajectoryBuilder(liftPosition)
                .lineToLinearHeading(new Pose2d(0, (nextToWall) * multiplier))
                .build();

        // go to intake return point
        final Trajectory part3 = drive.trajectoryBuilder(part2.end())
                .lineToLinearHeading(intakeReturnPoint)
                .build();
        // hello guys its me fin I am going to blow up GitHub HQ also idk how to use shift key
        // lololololol kek derp face
        // part 4: move back to Alliance Shipping hub. then you can go back to part 2 as needed.
        final TrajectorySequence part4 = drive.trajectorySequenceBuilder(intakeReturnPoint)
                .lineTo(new Vector2d(-3, nextToWall * multiplier))
                .lineToLinearHeading(isRed ? new Pose2d(-4, -44, Math.toRadians(-62)) :
                        new Pose2d(-3, 45, Math.toRadians(60)))
                .build();

        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
            }
        });

        AutoLED led = new AutoLED(hardwareMap, detector);
        waitForStart();
        led.stop();
        toolTimer.reset();
        liftThread.start();
        eventThread.start();

        intake.lightsOff();
        height = detector.run();
        intake.lightsOn();
        goodTelemetry.addData("height", height);
        goodTelemetry.update();

        DelayStorage.waitForDelay(toolTimer);

        drive.followTrajectorySequenceAsync(part1);
        updateLoop(drive);
        lift.setPosition(getPosition(height));
        toolTimer.reset();
        while (lift.getPosition() != AutoLift.Positions.INTAKING && /* Ethan doodoohead */ toolTimer.seconds() < 5) {
            if (isStopRequested()) {
                return;
            }
            stayInPose(drive, part1.end());
        }

        drive.followTrajectoryAsync(part2);
        updateLoop(drive);
        if (isStopRequested()) return;

        WallSmash.smashIntoWallSideways(drive, multiplier, 250);
        driveToPose(drive, part2.end());

        drive.followTrajectoryAsync(part3);
        updateLoop(drive);
        if (isStopRequested()) return;

        intake(toolTimer, drive, intake);

        driveToPose(drive, intakeReturnPoint);
        updateLoop(drive);
        if (isStopRequested()) return;
        WallSmash.smashIntoWallSideways(drive, multiplier, 500);
        driveToPose(drive, intakeReturnPoint);

        drive.followTrajectorySequenceAsync(part4);
        updateLoop(drive);
        if (isStopRequested()) return;

        lift.setPosition(AutoLift.Positions.TOP);
        toolTimer.reset();
        while (lift.getPosition() != AutoLift.Positions.INTAKING && /* Ethan doodoohead */ toolTimer.seconds() < 5) {
            if (isStopRequested()) {
                return;
            }
            stayInPose(drive, part4.end());
        }

        drive.followTrajectoryAsync(part2);
        updateLoop(drive);

        WallSmash.smashIntoWallSideways(drive, multiplier, 250);
        driveToPose(drive, part2.end());
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(part3);
        updateLoop(drive);
        if (isStopRequested()) return;

        intake(toolTimer, drive, intake);
        Pose2d endingPose = drive.getPoseEstimate();
        while (!isStopRequested()) {
            stayInPose(drive, endingPose);
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void intake(ElapsedTime timer, @NonNull SampleMecanumDrive drive, @NonNull AutoIntake intake) {
        // intake a block
        intake.backward();
        drive.setWeightedDrivePower(new Pose2d(0.4, 0, 0));
        while (intake.noObject()) {
            if (isStopRequested()) {
                return;
            }
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        timer.reset();
        while (timer.milliseconds() < 250) { }
        intake.stop();
        intake.forward();
        timer.reset();
        while (timer.milliseconds() < 100) {}
        intake.stop();
    }

    public void updateLoop(SampleMecanumDrive drive) {
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }

    public void driveToPose(@NonNull SampleMecanumDrive drive, Pose2d pose) {
        if (!drive.getPoseEstimate().epsilonEquals(pose)) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(pose)
                    .build());

            while (!isStopRequested() && drive.isBusy()) {
                drive.update();
            }
        }
    }
}
